#import "server.h"

#import <Metal/Metal.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "common.h"
#include "compositor.h"
#include "encoder.h"
#include "metalvr_math.h"
#include "shared_texture.h"

namespace metalvr {

namespace {

bool readFull(int fd, void *data, size_t size) {
  auto *p = static_cast<uint8_t *>(data);
  while (size) {
    ssize_t n = ::recv(fd, p, size, 0);
    if (n == 0)
      return false;
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    p += n;
    size -= static_cast<size_t>(n);
  }
  return true;
}

bool writeFull(int fd, const void *data, size_t size) {
  auto *p = static_cast<const uint8_t *>(data);
  while (size) {
    ssize_t n = ::send(fd, p, size, 0);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    p += n;
    size -= static_cast<size_t>(n);
  }
  return true;
}

MTLPixelFormat metalFormat(uint32_t format) {
  switch (format) {
  case MVR_FORMAT_RGBA8_UNORM:
    return MTLPixelFormatRGBA8Unorm;
  case MVR_FORMAT_RGBA8_SRGB:
    return MTLPixelFormatRGBA8Unorm_sRGB;
  case MVR_FORMAT_BGRA8_UNORM:
    return MTLPixelFormatBGRA8Unorm;
  case MVR_FORMAT_BGRA8_SRGB:
    return MTLPixelFormatBGRA8Unorm_sRGB;
  case MVR_FORMAT_RGBA16_FLOAT:
    return MTLPixelFormatRGBA16Float;
  case MVR_FORMAT_RGB10A2_UNORM:
    return MTLPixelFormatRGB10A2Unorm;
  case MVR_FORMAT_RGBA16_UNORM:
    return MTLPixelFormatRGBA16Unorm;
  case MVR_FORMAT_BGRX8_UNORM:
    return MTLPixelFormatBGRA8Unorm;
  case MVR_FORMAT_BGRX8_SRGB:
    return MTLPixelFormatBGRA8Unorm_sRGB;
  default:
    return MTLPixelFormatInvalid;
  }
}

struct Swapchain {
  uint32_t width = 0, height = 0, array_size = 1, image_count = 0;
  std::vector<id<MTLTexture>> textures; // image * array_size + slice
};

struct Client {
  int fd = -1;
  uint64_t id = 0;
  bool greeted = false;
  std::string name;
  std::map<uint32_t, Swapchain> swapchains;
};

} // namespace

struct Server::Impl {
  std::unique_ptr<Driver> driver;
  ServerOptions options;
  id<MTLDevice> device = nil;
  std::unique_ptr<Compositor> compositor;
  std::unique_ptr<Encoder> encoder;

  int listen_fd = -1;
  std::atomic<bool> stopping{false};
  std::atomic<uint64_t> next_client{1};

  std::mutex mutex; // session ownership, pacing, open connections
  uint64_t session_owner = 0;
  bool session_seen = false;
  int64_t last_wake_ns = 0;
  uint64_t next_frame = 1;
  bool keyframe_wanted = true;
  uint64_t late_frames = 0; // frame waits that arrived after their slot on the refresh grid
  std::set<int> open_fds;

  std::mutex frame_mutex; // one frame is composed at a time
  uint64_t composed = 0;
  int64_t compose_ns_total = 0;
  int64_t encode_ns_total = 0; // time spent handing frames to the encoder, which blocks when it is behind
  int64_t last_report_ns = 0;
  uint64_t last_report_frames = 0;

  void onEncoded(EncodedFrame &&frame) {
    if (!frame.config.empty())
      driver->sendVideoConfig(frame.info.codec, frame.config.data(), frame.config.size());
    driver->sendFrame(frame.info, frame.data.data(), frame.data.size());
  }

  template <typename T> bool respond(Client &client, uint16_t type, const T &payload) {
    mvr_header header{MVR_MAGIC, static_cast<uint16_t>(type | MVR_RESPONSE_BIT), 0, sizeof(T)};
    uint8_t buffer[sizeof(mvr_header) + sizeof(T)];
    std::memcpy(buffer, &header, sizeof(header));
    std::memcpy(buffer + sizeof(header), &payload, sizeof(T));
    return writeFull(client.fd, buffer, sizeof(buffer));
  }

  bool respondResult(Client &client, uint16_t type, uint32_t result) {
    return respond(client, type, mvr_result_response{result});
  }

  uint32_t createSwapchain(Client &client, const mvr_swapchain_create_request &request) {
    if (!request.image_count || !request.array_size || !request.width || !request.height ||
        request.array_size > MVR_MAX_SWAPCHAIN_IMAGES || request.image_count > MVR_MAX_SWAPCHAIN_IMAGES ||
        request.image_count * request.array_size > MVR_MAX_SWAPCHAIN_IMAGES)
      return MVR_ERR_INVALID;
    if (client.swapchains.count(request.swapchain_id))
      return MVR_ERR_INVALID;
    const MTLPixelFormat format = metalFormat(request.format);
    Swapchain swapchain;
    swapchain.width = request.width;
    swapchain.height = request.height;
    swapchain.array_size = request.array_size;
    swapchain.image_count = request.image_count;
    const uint32_t shares = request.image_count * request.array_size;
    for (uint32_t i = 0; i < shares; ++i) {
      id<MTLTexture> texture = nil;
      if (request.share_kind == MVR_SHARE_MULLION_BOOTSTRAP) {
        char name[MVR_SHARE_NAME_LEN + 1] = {};
        std::memcpy(name, request.images[i].name, MVR_SHARE_NAME_LEN);
        texture = openMullionSharedTexture(device, name);
      } else if (request.share_kind == MVR_SHARE_IOSURFACE_ID) {
        if (format == MTLPixelFormatInvalid)
          return MVR_ERR_INVALID;
        texture = openIOSurfaceTexture(device, request.images[i].iosurface_id, format);
      } else {
        return MVR_ERR_INVALID;
      }
      if (!texture)
        return MVR_ERR_SHARE;
      if (texture.textureType != MTLTextureType2D || texture.width != request.width ||
          texture.height != request.height) {
        logInfo("swapchain %u image %u: the shared texture is %lux%lu type %lu, not the %ux%u 2D texture declared",
                request.swapchain_id, i, static_cast<unsigned long>(texture.width),
                static_cast<unsigned long>(texture.height), static_cast<unsigned long>(texture.textureType),
                request.width, request.height);
        return MVR_ERR_SHARE;
      }
      swapchain.textures.push_back(texture);
    }
    logDebug("client %llu: swapchain %u %ux%u, %u images x %u slices, Metal format %lu",
             static_cast<unsigned long long>(client.id), request.swapchain_id, request.width, request.height,
             request.image_count, request.array_size,
             static_cast<unsigned long>(swapchain.textures.front().pixelFormat));
    client.swapchains.emplace(request.swapchain_id, std::move(swapchain));
    return MVR_OK;
  }

  mvr_frame_wait_response waitFrame() {
    mvr_frame_wait_response response{};
    const mvr_system_info info = driver->systemInfo();
    const float hz = info.refresh_hz > 1.0f ? info.refresh_hz : 72.0f;
    const int64_t period = static_cast<int64_t>(1e9 / hz);
    const int64_t now = nowNs();
    std::lock_guard<std::mutex> lock(mutex);
    int64_t wake = last_wake_ns + period;
    // an application that fell behind starts its next frame now, and the grid restarts from there
    if (wake < now) {
      if (last_wake_ns)
        ++late_frames;
      wake = now;
    }
    last_wake_ns = wake;
    response.result = MVR_OK;
    response.should_render = driver->connected() ? 1 : 0;
    response.frame_id = next_frame++;
    response.period_ns = period;
    response.wake_time_ns = wake;
    // a frame begun at its wake time is on the headset's display about two periods later: one to render, one to
    // encode, send and decode. The headset reprojects from the pose the frame carries, so this only has to be close.
    response.predicted_display_time_ns = wake + 2 * period;
    return response;
  }

  uint32_t endFrame(Client &client, const mvr_frame_end_request &request, const mvr_layer *layers) {
    const int64_t submitted_ns = nowNs();
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (session_owner != client.id)
        return MVR_ERR_STATE;
    }
    if (!request.layer_count || !driver->connected())
      return MVR_OK;

    ComposeFrame frame;
    bool have_eyes = false;
    for (uint32_t i = 0; i < request.layer_count; ++i) {
      const mvr_layer &in = layers[i];
      ComposeLayer layer;
      layer.type = in.type;
      layer.flags = in.flags;
      layer.eye_mask = in.eye_mask ? in.eye_mask : 3;
      layer.quad_pose = in.quad_pose;
      layer.quad_width = in.quad_width;
      layer.quad_height = in.quad_height;
      const uint32_t views = in.type == MVR_LAYER_PROJECTION ? 2 : in.type == MVR_LAYER_QUAD ? 1 : 0;
      if (!views)
        return MVR_ERR_INVALID;
      for (uint32_t v = 0; v < views; ++v) {
        const mvr_layer_view &view = in.views[v];
        auto found = client.swapchains.find(view.swapchain_id);
        if (found == client.swapchains.end())
          return MVR_ERR_INVALID;
        const Swapchain &swapchain = found->second;
        if (view.image_index >= swapchain.image_count || view.array_index >= swapchain.array_size)
          return MVR_ERR_INVALID;
        // a mirrored rect is checked by the region it covers
        const int64_t x0 = std::min<int64_t>(view.rect.x, static_cast<int64_t>(view.rect.x) + view.rect.width);
        const int64_t x1 = std::max<int64_t>(view.rect.x, static_cast<int64_t>(view.rect.x) + view.rect.width);
        const int64_t y0 = std::min<int64_t>(view.rect.y, static_cast<int64_t>(view.rect.y) + view.rect.height);
        const int64_t y1 = std::max<int64_t>(view.rect.y, static_cast<int64_t>(view.rect.y) + view.rect.height);
        if (!view.rect.width || !view.rect.height || x0 < 0 || y0 < 0 || x1 > swapchain.width || y1 > swapchain.height)
          return MVR_ERR_INVALID;
        layer.views[v].texture = swapchain.textures[view.image_index * swapchain.array_size + view.array_index];
        layer.views[v].rect = view.rect;
      }
      if (in.type == MVR_LAYER_PROJECTION && !have_eyes) {
        for (int eye = 0; eye < 2; ++eye) {
          frame.eye_pose[eye] = in.views[eye].pose;
          frame.eye_fov[eye] = in.views[eye].fov;
        }
        have_eyes = true;
      }
      frame.layers.push_back(std::move(layer));
    }
    if (!have_eyes) {
      // only quads: place them with the newest poses
      mvr_tracking_response tracking{};
      driver->tracking(request.display_time_ns, tracking);
      for (int eye = 0; eye < 2; ++eye) {
        frame.eye_pose[eye] = tracking.views[eye].pose;
        frame.eye_fov[eye] = tracking.views[eye].fov;
      }
    }
    frame.head_pose.orientation = frame.eye_pose[0].orientation;
    frame.head_pose.position = (frame.eye_pose[0].position + frame.eye_pose[1].position) * 0.5f;

    std::lock_guard<std::mutex> lock(frame_mutex);
    const VideoParams params = driver->videoParams();
    if (!params.frame_width || !params.frame_height)
      return MVR_OK;
    const bool displays = driver->displaysFrames();
    if (!compositor->configure(params.frame_width, params.frame_height) || (!displays && !encoder->configure(params)))
      return MVR_ERR_INTERNAL;

    const int64_t begin = nowNs();
    CVPixelBufferRef buffer = compositor->compose(frame);
    if (!buffer)
      return MVR_ERR_INTERNAL;
    compose_ns_total += nowNs() - begin;
    ++composed;

    if (!options.snapshot_path.empty() && composed == options.snapshot_frame) {
      if (writePixelBufferPNG(buffer, options.snapshot_path.c_str()))
        logInfo("wrote composed frame %llu to %s", static_cast<unsigned long long>(composed),
                options.snapshot_path.c_str());
      else
        logInfo("could not write %s", options.snapshot_path.c_str());
    }
    if (g_snapshot_requested) {
      g_snapshot_requested = 0;
      const char *home = std::getenv("HOME");
      const std::string path = std::string(home ? home : "/tmp") + "/.metalsharp/logs/metalvr-snapshot.png";
      logInfo(writePixelBufferPNG(buffer, path.c_str()) ? "wrote the composed frame to %s" : "could not write %s",
              path.c_str());
      for (size_t l = 0; l < frame.layers.size(); ++l)
        for (int eye = 0; eye < 2; ++eye) {
          const ComposeLayerView &v = frame.layers[l].views[eye];
          logInfo("snapshot: layer %zu eye %d: type %u flags %x, texture %s %lux%lu format %lu storage %lu, rect %d,%d "
                  "%dx%d, fov %.3f %.3f %.3f %.3f",
                  l, eye, frame.layers[l].type, frame.layers[l].flags, v.texture ? "yes" : "none",
                  (unsigned long)v.texture.width, (unsigned long)v.texture.height,
                  (unsigned long)v.texture.pixelFormat, (unsigned long)v.texture.storageMode, v.rect.x, v.rect.y,
                  v.rect.width, v.rect.height, frame.eye_fov[eye].left, frame.eye_fov[eye].right,
                  frame.eye_fov[eye].up, frame.eye_fov[eye].down);
          const std::string raw = std::string(home ? home : "/tmp") + "/.metalsharp/logs/metalvr-snapshot-layer" +
                                  std::to_string(l) + "-eye" + std::to_string(eye) + ".png";
          if (v.texture && !compositor->writeTexturePNG(v.texture, raw.c_str()))
            logInfo("snapshot: could not write %s", raw.c_str());
        }
      // every image the client shares, shown or not: with SteamVR an application's own eye images are swapchains
      // of their own, which its compositor draws from
      for (auto &entry : client.swapchains)
        for (size_t i = 0; i < entry.second.textures.size(); ++i) {
          id<MTLTexture> texture = entry.second.textures[i];
          const std::string raw = std::string(home ? home : "/tmp") + "/.metalsharp/logs/metalvr-snapshot-swapchain" +
                                  std::to_string(entry.first) + "-" + std::to_string(i) + ".png";
          double color = 0, alpha = 0;
          const bool written = texture && compositor->writeTexturePNG(texture, raw.c_str(), &color, &alpha);
          logInfo("snapshot: swapchain %u image %zu: %lux%lu format %lu, mean colour %.2f alpha %.2f of 255%s",
                  entry.first, i, (unsigned long)texture.width, (unsigned long)texture.height,
                  (unsigned long)texture.pixelFormat, color, alpha, written ? "" : " (not written)");
        }
    }

    if (displays) {
      // a headset on the Mac's display: the driver draws the composed frame itself
      FrameInfo info;
      info.sample_id = request.sample_id;
      info.display_time_ns = request.display_time_ns;
      info.submitted_ns = submitted_ns;
      info.composed_ns = nowNs();
      info.head_orientation = frame.head_pose.orientation;
      info.has_orientation = have_eyes;
      driver->displayFrame(info, buffer);
      CVPixelBufferRelease(buffer);
      return MVR_OK;
    }

    bool keyframe = driver->consumeKeyframeRequest();
    {
      std::lock_guard<std::mutex> state(mutex);
      keyframe = keyframe || keyframe_wanted;
      keyframe_wanted = false;
    }
    const int64_t encode_begin = nowNs();
    FrameInfo info;
    info.sample_id = request.sample_id;
    info.display_time_ns = request.display_time_ns;
    info.submitted_ns = submitted_ns;
    info.composed_ns = encode_begin;
    encoder->encode(buffer, info, keyframe);
    encode_ns_total += nowNs() - encode_begin;
    CVPixelBufferRelease(buffer);

    const int64_t now = nowNs();
    if (g_verbose && now - last_report_ns > 5'000'000'000) {
      const uint64_t frames = composed - last_report_frames;
      const Encoder::Stats encoded = encoder->takeStats();
      uint64_t late = 0;
      {
        std::lock_guard<std::mutex> state(mutex);
        late = late_frames;
        late_frames = 0;
      }
      if (last_report_ns && frames)
        logDebug("%.1f frames/s (%llu late), compose %.2f ms, encoder submit %.2f ms, encode %.2f ms (max %.2f), "
                 "%llu dropped",
                 frames / ((now - last_report_ns) / 1e9), static_cast<unsigned long long>(late),
                 compose_ns_total / 1e6 / frames, encode_ns_total / 1e6 / frames, encoded.mean_latency_ms,
                 encoded.max_latency_ms, static_cast<unsigned long long>(encoded.dropped_total));
      last_report_ns = now;
      last_report_frames = composed;
      compose_ns_total = 0;
      encode_ns_total = 0;
    }
    return MVR_OK;
  }

  // false when the connection should close
  bool dispatch(Client &client, const mvr_header &header, const std::vector<uint8_t> &payload) {
    const auto as = [&](auto &out) {
      if (payload.size() < sizeof(out))
        return false;
      std::memcpy(&out, payload.data(), sizeof(out));
      return true;
    };

    if (header.type != MVR_MSG_HELLO && !client.greeted)
      return false;

    switch (header.type) {
    case MVR_MSG_HELLO: {
      mvr_hello_request request{};
      mvr_hello_response response{};
      response.version = MVR_PROTOCOL_VERSION;
      if (!as(request) || request.version != MVR_PROTOCOL_VERSION) {
        response.result = MVR_ERR_VERSION;
        respond(client, header.type, response);
        return false;
      }
      request.application_name[sizeof(request.application_name) - 1] = 0;
      client.name = request.application_name;
      client.greeted = true;
      response.result = MVR_OK;
      response.server_time_ns = nowNs();
      response.system = driver->systemInfo();
      logInfo("client %llu: '%s' (pid %u, %s)", static_cast<unsigned long long>(client.id), client.name.c_str(),
              request.pid,
              request.graphics_api == MVR_GFX_D3D11   ? "Direct3D 11"
              : request.graphics_api == MVR_GFX_D3D12 ? "Direct3D 12"
              : request.graphics_api == MVR_GFX_METAL ? "Metal"
                                                      : "no graphics");
      return respond(client, header.type, response);
    }
    case MVR_MSG_SYSTEM: {
      mvr_system_response response{};
      response.system = driver->systemInfo();
      return respond(client, header.type, response);
    }
    case MVR_MSG_REFERENCES: {
      mvr_references_response response{};
      response.count = std::min<uint32_t>(driver->trackingReferences(response.references),
                                          MVR_MAX_TRACKING_REFERENCES);
      return respond(client, header.type, response);
    }
    case MVR_MSG_CLOCK: {
      mvr_clock_response response{};
      response.server_time_ns = nowNs();
      return respond(client, header.type, response);
    }
    case MVR_MSG_SESSION_BEGIN: {
      uint32_t result = MVR_OK;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (session_owner && session_owner != client.id) {
          result = MVR_ERR_BUSY;
        } else {
          session_owner = client.id;
          session_seen = true;
          last_wake_ns = 0;
          keyframe_wanted = true;
        }
      }
      if (result == MVR_OK)
        logInfo("client %llu: session running", static_cast<unsigned long long>(client.id));
      return respondResult(client, header.type, result);
    }
    case MVR_MSG_SESSION_END: {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (session_owner == client.id)
          session_owner = 0;
      }
      logInfo("client %llu: session ended", static_cast<unsigned long long>(client.id));
      return respondResult(client, header.type, MVR_OK);
    }
    case MVR_MSG_SWAPCHAIN_CREATE: {
      mvr_swapchain_create_request request{};
      if (!as(request))
        return false;
      return respondResult(client, header.type, createSwapchain(client, request));
    }
    case MVR_MSG_SWAPCHAIN_DESTROY: {
      mvr_swapchain_destroy_request request{};
      if (!as(request))
        return false;
      {
        // a frame being composed may still hold the textures; they go when it is done
        std::lock_guard<std::mutex> lock(frame_mutex);
        client.swapchains.erase(request.swapchain_id);
      }
      return respondResult(client, header.type, MVR_OK);
    }
    case MVR_MSG_FRAME_WAIT:
      return respond(client, header.type, waitFrame());
    case MVR_MSG_TRACKING: {
      mvr_tracking_request request{};
      if (!as(request))
        return false;
      mvr_tracking_response response{};
      driver->tracking(request.time_ns, response);
      response.result = MVR_OK;
      return respond(client, header.type, response);
    }
    case MVR_MSG_FRAME_END: {
      mvr_frame_end_request request{};
      if (!as(request) || request.layer_count > MVR_MAX_LAYERS ||
          payload.size() < sizeof(request) + request.layer_count * sizeof(mvr_layer))
        return false;
      std::vector<mvr_layer> layers(request.layer_count);
      if (request.layer_count)
        std::memcpy(layers.data(), payload.data() + sizeof(request), request.layer_count * sizeof(mvr_layer));
      return respondResult(client, header.type, endFrame(client, request, layers.data()));
    }
    case MVR_MSG_HAPTIC: {
      mvr_haptic_request request{};
      if (!as(request) || request.hand > 1)
        return false;
      driver->haptic(request.hand, request.duration_s, request.frequency_hz, request.amplitude);
      return respondResult(client, header.type, MVR_OK);
    }
    default:
      return false;
    }
  }

  void serve(int fd) {
    @autoreleasepool {
      Client client;
      client.fd = fd;
      client.id = next_client++;
      std::vector<uint8_t> payload;
      for (;;) {
        mvr_header header{};
        if (!readFull(fd, &header, sizeof(header)))
          break;
        if (header.magic != MVR_MAGIC || header.size > MVR_MAX_PAYLOAD)
          break;
        payload.resize(header.size);
        if (header.size && !readFull(fd, payload.data(), header.size))
          break;
        @autoreleasepool {
          if (!dispatch(client, header, payload))
            break;
        }
      }

      bool owned = false;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (session_owner == client.id) {
          session_owner = 0;
          owned = true;
        }
        open_fds.erase(fd);
      }
      {
        std::lock_guard<std::mutex> lock(frame_mutex);
        client.swapchains.clear();
      }
      ::close(fd);
      if (client.greeted)
        logInfo("client %llu: disconnected", static_cast<unsigned long long>(client.id));
      if (owned && options.exit_after_session)
        requestStopLocked();
    }
  }

  void requestStopLocked() { stopping = true; }
};

Server::Server(std::unique_ptr<Driver> driver, const ServerOptions &options) : impl_(new Impl) {
  impl_->driver = std::move(driver);
  impl_->options = options;
}

Server::~Server() {
  if (impl_->listen_fd >= 0)
    ::close(impl_->listen_fd);
  if (impl_->encoder)
    impl_->encoder->flush();
  impl_->encoder.reset();
  if (impl_->driver)
    impl_->driver->stop();
}

bool Server::start() {
  Impl &s = *impl_;
  s.device = MTLCreateSystemDefaultDevice();
  if (!s.device) {
    logInfo("no Metal device");
    return false;
  }
  s.compositor = std::make_unique<Compositor>(s.device);
  Impl *impl = impl_.get();
  s.encoder = std::make_unique<Encoder>([impl](EncodedFrame &&frame) { impl->onEncoded(std::move(frame)); });
  s.encoder->setSoftware(s.options.software_encoder);

  if (!s.driver->start()) {
    logInfo("driver '%s' did not start", s.driver->name());
    return false;
  }

  s.listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (s.listen_fd < 0)
    return false;
  int one = 1;
  ::setsockopt(s.listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(s.options.port);
  // runtimes are on this machine; nothing else may connect
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(s.listen_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    logInfo("cannot bind 127.0.0.1:%u: %s (is another metalvrd running?)", s.options.port, std::strerror(errno));
    return false;
  }
  if (::listen(s.listen_fd, 8) != 0)
    return false;
  const mvr_system_info info = s.driver->systemInfo();
  logInfo("listening on 127.0.0.1:%u, driver '%s' (%s, %ux%u per eye, %.0f Hz)", s.options.port, s.driver->name(),
          info.system_name, info.eye_width, info.eye_height, info.refresh_hz);
  return true;
}

void Server::run() {
  Impl &s = *impl_;
  while (!s.stopping) {
    // accept is not woken by another thread, so the stop flag is polled
    pollfd waiting{s.listen_fd, POLLIN, 0};
    int ready = ::poll(&waiting, 1, 200);
    if (ready < 0 && errno != EINTR)
      break;
    if (ready <= 0)
      continue;
    int fd = ::accept(s.listen_fd, nullptr, nullptr);
    if (fd < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
    {
      std::lock_guard<std::mutex> lock(s.mutex);
      s.open_fds.insert(fd);
    }
    Impl *impl = impl_.get();
    std::thread([impl, fd] { impl->serve(fd); }).detach();
  }
  // connections still open end here; their threads see the close and leave
  std::set<int> fds;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    fds = s.open_fds;
  }
  for (int fd : fds)
    ::shutdown(fd, SHUT_RDWR);
  for (int i = 0; i < 100; ++i) {
    {
      std::lock_guard<std::mutex> lock(s.mutex);
      if (s.open_fds.empty())
        break;
    }
    ::usleep(10'000);
  }
  if (s.encoder)
    s.encoder->flush();
}

void Server::requestStop() { impl_->requestStopLocked(); }

} // namespace metalvr
