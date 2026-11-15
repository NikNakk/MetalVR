#import "encoder.h"

#import <CoreMedia/CoreMedia.h>
#import <Foundation/Foundation.h>
#import <VideoToolbox/VideoToolbox.h>

#include <memory>
#include <mutex>

#include "common.h"

namespace metalvr {

namespace {

const uint8_t kStartCode[4] = {0, 0, 0, 1};

struct FrameContext {
  FrameInfo info;
  int64_t queued_ns; // when it went into the encoder
};

// the stream's parameter sets, each behind a start code
std::vector<uint8_t> parameterSets(CMFormatDescriptionRef format, Codec codec) {
  std::vector<uint8_t> out;
  size_t count = 0;
  int nal_length_size = 0;
  OSStatus status = codec == Codec::HEVC
                        ? CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format, 0, nullptr, nullptr, &count,
                                                                             &nal_length_size)
                        : CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, nullptr, nullptr, &count,
                                                                             &nal_length_size);
  if (status != noErr)
    return out;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t *set = nullptr;
    size_t size = 0;
    status = codec == Codec::HEVC
                 ? CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format, i, &set, &size, nullptr, nullptr)
                 : CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, i, &set, &size, nullptr, nullptr);
    if (status != noErr || !set)
      continue;
    out.insert(out.end(), kStartCode, kStartCode + 4);
    out.insert(out.end(), set, set + size);
  }
  return out;
}

} // namespace

struct Encoder::Impl {
  Output output;
  std::mutex mutex;
  VTCompressionSessionRef session = nullptr;
  VideoParams params;
  bool configured = false;
  bool software = false;          // asked for
  bool session_software = false;  // what the current session is
  int64_t frame_index = 0;
  std::vector<uint8_t> last_config;

  std::mutex stats_mutex;
  uint64_t stats_frames = 0;
  int64_t stats_latency_total_ns = 0;
  int64_t stats_latency_max_ns = 0;
  uint64_t stats_dropped = 0; // never reset: the total the encoder gave up on
  int64_t last_drop_report_ns = 0;

  void destroy() {
    if (session) {
      VTCompressionSessionCompleteFrames(session, kCMTimeInvalid);
      VTCompressionSessionInvalidate(session);
      CFRelease(session);
      session = nullptr;
    }
    configured = false;
  }

  void onOutput(FrameContext *context, OSStatus status, VTEncodeInfoFlags flags, CMSampleBufferRef sample) {
    std::unique_ptr<FrameContext> owned(context);
    if (status != noErr || (flags & kVTEncodeInfo_FrameDropped) || !sample || !CMSampleBufferDataIsReady(sample)) {
      // a frame the encoder gave up on never reaches the headset; say so, but not once per frame
      std::lock_guard<std::mutex> lock(stats_mutex);
      ++stats_dropped;
      const int64_t now = nowNs();
      if (now - last_drop_report_ns > 1'000'000'000) {
        last_drop_report_ns = now;
        logInfo("encoder: frames are being dropped (%llu so far, status %d)",
                static_cast<unsigned long long>(stats_dropped), static_cast<int>(status));
      }
      return;
    }
    {
      const int64_t latency = nowNs() - owned->queued_ns;
      std::lock_guard<std::mutex> lock(stats_mutex);
      ++stats_frames;
      stats_latency_total_ns += latency;
      if (latency > stats_latency_max_ns)
        stats_latency_max_ns = latency;
    }
    EncodedFrame frame;
    frame.info = owned->info;
    frame.info.encoded_ns = nowNs();

    frame.info.keyframe = true;
    if (CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sample, false)) {
      if (CFArrayGetCount(attachments) > 0) {
        auto dict = static_cast<CFDictionaryRef>(CFArrayGetValueAtIndex(attachments, 0));
        CFBooleanRef not_sync = nullptr;
        if (CFDictionaryGetValueIfPresent(dict, kCMSampleAttachmentKey_NotSync,
                                          reinterpret_cast<const void **>(&not_sync)) &&
            not_sync && CFBooleanGetValue(not_sync))
          frame.info.keyframe = false;
      }
    }

    // the codec the frame went in with: the session's may have changed since
    const Codec codec = frame.info.codec;
    CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sample);
    int nal_length_size = 4;
    if (format) {
      if (codec == Codec::HEVC)
        CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format, 0, nullptr, nullptr, nullptr, &nal_length_size);
      else
        CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, nullptr, nullptr, nullptr, &nal_length_size);
    }
    if (frame.info.keyframe && format) {
      std::vector<uint8_t> config = parameterSets(format, codec);
      if (!config.empty() && config != last_config) {
        last_config = config;
        frame.config = std::move(config);
      }
    }

    CMBlockBufferRef block = CMSampleBufferGetDataBuffer(sample);
    const size_t total = block ? CMBlockBufferGetDataLength(block) : 0;
    std::vector<uint8_t> raw(total);
    if (!block || CMBlockBufferCopyDataBytes(block, 0, total, raw.data()) != kCMBlockBufferNoErr)
      return;
    // length-prefixed NAL units to start codes
    frame.data.reserve(total + 16);
    size_t offset = 0;
    while (offset + nal_length_size <= total) {
      size_t length = 0;
      for (int i = 0; i < nal_length_size; ++i)
        length = (length << 8) | raw[offset + i];
      offset += nal_length_size;
      if (length > total - offset)
        break;
      frame.data.insert(frame.data.end(), kStartCode, kStartCode + 4);
      frame.data.insert(frame.data.end(), raw.begin() + offset, raw.begin() + offset + length);
      offset += length;
    }
    if (output)
      output(std::move(frame));
  }
};

namespace {

void compressionCallback(void *refcon, void *frame_refcon, OSStatus status, VTEncodeInfoFlags flags,
                         CMSampleBufferRef sample) {
  static_cast<Encoder::Impl *>(refcon)->onOutput(static_cast<FrameContext *>(frame_refcon), status, flags, sample);
}

void setProperty(VTCompressionSessionRef session, CFStringRef key, CFTypeRef value) {
  OSStatus status = VTSessionSetProperty(session, key, value);
  if (status != noErr)
    logDebug("encoder: property %s not applied (%d)", CFStringGetCStringPtr(key, kCFStringEncodingUTF8) ?: "?",
             static_cast<int>(status));
}

} // namespace

Encoder::Encoder(Output output) : impl_(new Impl) { impl_->output = std::move(output); }

void Encoder::setSoftware(bool software) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->software = software;
}

Encoder::~Encoder() {
  impl_->destroy();
  delete impl_;
}

bool Encoder::configure(const VideoParams &params) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Impl &s = *impl_;
  const bool same_session = s.configured && s.session_software == s.software && s.params.codec == params.codec &&
                            s.params.frame_width == params.frame_width &&
                            s.params.frame_height == params.frame_height && s.params.fps == params.fps;
  if (same_session) {
    if (s.params.bitrate_bps != params.bitrate_bps) {
      s.params.bitrate_bps = params.bitrate_bps;
      setProperty(s.session, kVTCompressionPropertyKey_AverageBitRate,
                  (__bridge CFTypeRef) @(static_cast<long long>(params.bitrate_bps)));
    }
    return true;
  }
  s.destroy();
  if (!params.frame_width || !params.frame_height)
    return false;

  // The plain hardware encoder, real time and with reordering off, is frame in / frame out. VideoToolbox's
  // low-latency rate control was measured against it and is not used: at a headset's 4288x2336 it took HEVC from
  // 14 ms a frame to 99 ms and dropped every H.264 frame, and at 2880x1584 it was no faster for either codec.
  // Hardware is required: a software encoder cannot keep a headset's rate, and falling back to one silently
  // would turn a missing encoder into a stream that stutters. The software encoder is only used when asked for.
  NSDictionary *specification =
      s.software ? @{(id)kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder : @NO}
                 : @{(id)kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder : @YES};
  const CMVideoCodecType type = params.codec == Codec::HEVC ? kCMVideoCodecType_HEVC : kCMVideoCodecType_H264;
  OSStatus status = VTCompressionSessionCreate(
      kCFAllocatorDefault, static_cast<int32_t>(params.frame_width), static_cast<int32_t>(params.frame_height), type,
      (__bridge CFDictionaryRef)specification, nullptr, kCFAllocatorDefault, compressionCallback, impl_, &s.session);
  if (status != noErr) {
    logInfo("encoder: no %s encoder for %ux%u (%d)", s.software ? "software" : "hardware", params.frame_width,
            params.frame_height, static_cast<int>(status));
    s.session = nullptr;
    return false;
  }

  setProperty(s.session, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue);
  setProperty(s.session, kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse);
  setProperty(s.session, kVTCompressionPropertyKey_ProfileLevel,
              params.codec == Codec::HEVC ? kVTProfileLevel_HEVC_Main_AutoLevel : kVTProfileLevel_H264_High_AutoLevel);
  setProperty(s.session, kVTCompressionPropertyKey_AverageBitRate,
              (__bridge CFTypeRef) @(static_cast<long long>(params.bitrate_bps)));
  setProperty(s.session, kVTCompressionPropertyKey_ExpectedFrameRate, (__bridge CFTypeRef) @(params.fps));
  // keyframes are sent when the headset asks for one, not on a timer
  setProperty(s.session, kVTCompressionPropertyKey_MaxKeyFrameInterval, (__bridge CFTypeRef) @(INT32_MAX));
  setProperty(s.session, kVTCompressionPropertyKey_ColorPrimaries, kCVImageBufferColorPrimaries_ITU_R_709_2);
  setProperty(s.session, kVTCompressionPropertyKey_TransferFunction, kCVImageBufferTransferFunction_ITU_R_709_2);
  setProperty(s.session, kVTCompressionPropertyKey_YCbCrMatrix, kCVImageBufferYCbCrMatrix_ITU_R_709_2);
  VTCompressionSessionPrepareToEncodeFrames(s.session);

  s.params = params;
  s.session_software = s.software;
  s.configured = true;
  s.frame_index = 0;
  s.last_config.clear();
  logInfo("encoder: %s %ux%u @ %.0f fps, %.1f Mbps%s", params.codec == Codec::HEVC ? "HEVC" : "H.264",
          params.frame_width, params.frame_height, params.fps, params.bitrate_bps / 1e6,
          s.software ? ", SOFTWARE encoder (testing only)" : "");
  return true;
}

bool Encoder::encode(CVPixelBufferRef buffer, const FrameInfo &info, bool force_keyframe) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Impl &s = *impl_;
  if (!s.configured)
    return false;
  const int32_t timescale = 90000;
  const CMTime pts = CMTimeMake(static_cast<int64_t>(s.frame_index * (timescale / (s.params.fps > 0 ? s.params.fps : 72.0f))), timescale);
  const bool key = force_keyframe || s.frame_index == 0;
  ++s.frame_index;
  NSDictionary *properties = key ? @{(id)kVTEncodeFrameOptionKey_ForceKeyFrame : @YES} : nil;
  auto *context = new FrameContext{info, nowNs()};
  context->info.codec = s.params.codec;
  OSStatus status = VTCompressionSessionEncodeFrame(s.session, buffer, pts, kCMTimeInvalid,
                                                    (__bridge CFDictionaryRef)properties, context, nullptr);
  if (status != noErr) {
    delete context;
    logInfo("encoder: encode failed (%d)", static_cast<int>(status));
    return false;
  }
  return true;
}

Encoder::Stats Encoder::takeStats() {
  std::lock_guard<std::mutex> lock(impl_->stats_mutex);
  Stats stats;
  stats.frames = impl_->stats_frames;
  if (stats.frames)
    stats.mean_latency_ms = impl_->stats_latency_total_ns / 1e6 / static_cast<double>(stats.frames);
  stats.max_latency_ms = impl_->stats_latency_max_ns / 1e6;
  stats.dropped_total = impl_->stats_dropped;
  impl_->stats_frames = 0;
  impl_->stats_latency_total_ns = 0;
  impl_->stats_latency_max_ns = 0;
  return stats;
}

void Encoder::flush() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->session)
    VTCompressionSessionCompleteFrames(impl_->session, kCMTimeInvalid);
}

} // namespace metalvr
