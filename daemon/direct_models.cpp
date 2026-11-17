// What the direct-display driver is told of the Rift S, the Lighthouse headsets and the PlayStation VR2
// (direct_backend.h).
#include <cmath>

#include "direct_backend.h"
#include "headsets.h"
#include "lighthouse_protocol.h"
#include "psvr2_protocol.h"
#include "rift_s_protocol.h"

namespace metalvr {

namespace {

void fromSpec(DirectModel &m, uint32_t headset) {
  const HeadsetSpec &spec = headsetSpec(headset);
  const mvr_headset_description *description = mvr_describe_headset(headset);
  m.headset = headset;
  m.controllers = description->controllers;
  m.name = description->name;
  m.refresh_hz = spec.default_refresh;
  m.refresh_rates.assign(spec.refresh_rates, spec.refresh_rates + spec.refresh_rate_count);
  m.eye_width = spec.eye_width;
  m.eye_height = spec.eye_height;
  m.fov[0] = spec.fov[0];
  m.fov[1] = spec.fov[1];
  m.ipd = spec.ipd;
}

} // namespace

DirectModel riftSModel() {
  DirectModel m;
  fromSpec(m, MVR_HEADSET_RIFT_S);
  m.display_vendor = rift_s::kDisplayVendor;
  m.display_products = {rift_s::kDisplayProduct};
  m.portrait_eyes = true;
  for (int eye = 0; eye < 2; eye++) {
    rift_s::fieldOfView(eye, m.fov[eye].left, m.fov[eye].right, m.fov[eye].up, m.fov[eye].down);
    const rift_s::Panotools p = rift_s::panotools(eye);
    DirectLens &lens = m.lens[eye];
    lens.model = DirectLens::Panotools;
    for (int i = 0; i < 5; i++) lens.pano_k[i] = p.distortion_k[i];
    for (int i = 0; i < 3; i++) lens.pano_aberration[i] = p.aberration_k[i];
    lens.pano_scale = p.scale;
    lens.pano_center[0] = p.lens_center[0];
    lens.pano_center[1] = p.lens_center[1];
    lens.pano_viewport[0] = p.viewport[0];
    lens.pano_viewport[1] = p.viewport[1];
  }
  return m;
}

DirectModel psvr2Model(const std::array<float, 8> &calibration) {
  DirectModel m;
  fromSpec(m, MVR_HEADSET_PSVR2);
  // Sony's vendor id; the PS VR2's own product code in its EDID is not known yet (docs/HEADSETS.md), so any Sony
  // display is taken, and named in the log
  m.display_vendor = psvr2::kDisplayVendor;
  m.display_products = {};
  for (int eye = 0; eye < 2; eye++) {
    m.fov[eye] = psvr2::fieldOfView(eye);
    DirectLens &lens = m.lens[eye];
    lens.model = DirectLens::Psvr2;
    lens.psvr2[0] = calibration[eye ? 2 : 0];
    lens.psvr2[1] = calibration[eye ? 3 : 1];
    lens.psvr2[2] = calibration[eye ? 6 : 4];
    lens.psvr2[3] = calibration[eye ? 7 : 5];
  }
  return m;
}

DirectModel lighthouseModel(const lighthouse::HeadsetConfig &config, int pro2_mode) {
  using namespace lighthouse;
  DirectModel m;
  fromSpec(m, headsetOf(config.model));
  switch (config.model) {
  case Model::Vive:
    m.display_vendor = 0x22d2; // "HVR"
    m.display_products = {0xaa01};
    break;
  case Model::VivePro:
    m.display_vendor = 0x22d2;
    m.display_products = {0xaa02, 0xaa03}; // which is the Vive Pro's is a guess (docs/HEADSETS.md)
    break;
  case Model::VivePro2:
    m.display_vendor = 0x22d2;
    m.display_products = {0xaa04};
    break;
  case Model::Index:
    m.display_vendor = 0x5996; // "VLV"
    m.display_products = {0x91a8};
    for (uint32_t p = 0x91b0; p <= 0x91bf; p++) m.display_products.push_back(p);
    break;
  case Model::None:
    break;
  }
  // the panel's pixels for one eye, about 1.2 times over, as the CV1's render size is
  const auto render = [](int pixels) { return static_cast<uint32_t>(std::lround(pixels * 1.2 / 16.0) * 16); };
  int eye_w = config.eye_width, eye_h = config.eye_height;
  if (config.model == Model::VivePro2) {
    const auto &modes = pro2Modes();
    const Pro2Mode &mode = modes[pro2_mode >= 0 && pro2_mode < static_cast<int>(modes.size()) ? pro2_mode : 2];
    eye_w = mode.width / 2;
    eye_h = mode.height;
    m.refresh_hz = mode.refresh_hz;
  }
  if (eye_w > 0 && eye_h > 0) {
    m.eye_width = render(eye_w);
    m.eye_height = render(eye_h);
  }
  for (int eye = 0; eye < 2; eye++) {
    m.fov[eye] = config.fov[eye];
    m.eye_rotation[eye] = config.eye_rotation[eye];
    const Distortion &d = config.distortion[eye];
    DirectLens &lens = m.lens[eye];
    lens.model = DirectLens::Vive;
    lens.vive_aspect = d.aspect_x_over_y;
    lens.vive_grow = d.grow_for_undistort;
    for (int c = 0; c < 3; c++) {
      lens.vive_center[c][0] = d.center[c][0];
      lens.vive_center[c][1] = d.center[c][1];
      for (int k = 0; k < 4; k++) lens.vive_coefficients[c][k] = d.coefficients[c][k];
    }
    lens.flip_y = config.model == Model::VivePro2;
  }
  if (config.lens_separation > 0.05f && config.lens_separation < 0.08f) m.ipd = config.lens_separation;
  return m;
}

} // namespace metalvr
