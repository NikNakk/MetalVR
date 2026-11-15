// An OpenVR application: drives OpenComposite's openvr_api.dll the way a small SteamVR Direct3D 11 game would,
// so the whole SteamVR path is exercised: OpenVR -> OpenComposite -> OpenXR -> MetalVR's runtime -> metalvrd.
//
//   ovr_smoke [--frames N] [--pattern] [--flipped] [--typeless] [--msaa] [--manifest Z:\path\actions.json]
//
// --pattern   the top quarter of each eye's texture is white and the left eighth green, so orientation shows
// --flipped   submit with bounds whose vMin > vMax, as games with bottom-up textures do
// --typeless  the game's textures are typeless; --msaa makes them 4x multisampled
// --manifest  use the action manifest instead of legacy controller state
//
// Built and run by scripts/test-openvr.sh against the OpenVR header OpenComposite carries.
#include <windows.h>
#include <d3d11.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include "openvr-1.16.8.h"

static void say(const char* fmt, ...) {
  va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); printf("\n"); fflush(stdout);
}

int main(int argc, char** argv) {
  int frames = 150;
  bool flipped = false, manifest = false, msaa = false, typeless = false, pattern = false;
  const char* manifest_path = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--frames") && i + 1 < argc) frames = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--flipped")) flipped = true;
    else if (!strcmp(argv[i], "--msaa")) msaa = true;
    else if (!strcmp(argv[i], "--typeless")) typeless = true;
    else if (!strcmp(argv[i], "--pattern")) pattern = true;
    else if (!strcmp(argv[i], "--manifest") && i + 1 < argc) { manifest = true; manifest_path = argv[++i]; }
  }
  say("step: VR_IsHmdPresent");
  say("VR_IsHmdPresent=%d", (int)vr::VR_IsHmdPresent());

  say("step: VR_Init");
  vr::EVRInitError err = vr::VRInitError_None;
  vr::IVRSystem* sys = vr::VR_Init(&err, vr::VRApplication_Scene);
  if (!sys) { say("VR_Init failed: %d (%s)", (int)err, vr::VR_GetVRInitErrorAsEnglishDescription(err)); return 1; }
  say("VR_Init ok");

  uint32_t w = 0, h = 0;
  sys->GetRecommendedRenderTargetSize(&w, &h);
  say("recommended size %ux%u", w, h);
  float l, r, t, b;
  sys->GetProjectionRaw(vr::Eye_Left, &l, &r, &t, &b);
  say("projection raw left eye: l=%f r=%f t=%f b=%f", l, r, t, b);
  vr::HmdMatrix34_t e = sys->GetEyeToHeadTransform(vr::Eye_Left);
  say("eye-to-head left: x offset %f (row0 %f %f %f)", e.m[0][3], e.m[0][0], e.m[0][1], e.m[0][2]);
  vr::HiddenAreaMesh_t mesh = sys->GetHiddenAreaMesh(vr::Eye_Left);
  say("hidden area mesh triangles: %u", mesh.unTriangleCount);
  char name[256] = {};
  sys->GetStringTrackedDeviceProperty(0, vr::Prop_TrackingSystemName_String, name, sizeof(name));
  say("tracking system: %s, display frequency %f", name,
      sys->GetFloatTrackedDeviceProperty(0, vr::Prop_DisplayFrequency_Float));

  vr::IVRCompositor* comp = vr::VRCompositor();
  if (!comp) { say("no compositor"); return 1; }

  if (manifest) {
    say("step: SetActionManifestPath %s", manifest_path);
    vr::EVRInputError ie = vr::VRInput()->SetActionManifestPath(manifest_path);
    say("SetActionManifestPath -> %d", (int)ie);
  }

  vr::VRActionSetHandle_t set = 0;
  vr::VRActionHandle_t a_fire = 0, a_jump = 0, a_move = 0, a_pose = 0, a_haptic = 0;
  if (manifest) {
    vr::IVRInput* in = vr::VRInput();
    say("handles: set %d fire %d jump %d move %d pose %d haptic %d", (int)in->GetActionSetHandle("/actions/main", &set),
        (int)in->GetActionHandle("/actions/main/in/fire", &a_fire), (int)in->GetActionHandle("/actions/main/in/jump", &a_jump),
        (int)in->GetActionHandle("/actions/main/in/move", &a_move),
        (int)in->GetActionHandle("/actions/main/in/hand_right", &a_pose),
        (int)in->GetActionHandle("/actions/main/out/haptic", &a_haptic));
  }
  say("step: D3D11CreateDevice");
  ID3D11Device* dev = nullptr; ID3D11DeviceContext* ctx = nullptr;
  HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &dev,
                                 nullptr, &ctx);
  if (FAILED(hr)) { say("D3D11CreateDevice failed 0x%08lx", (unsigned long)hr); return 1; }

  ID3D11Texture2D* tex[2] = {}; ID3D11RenderTargetView* rtv[2] = {};
  for (int eye = 0; eye < 2; ++eye) {
    D3D11_TEXTURE2D_DESC d{};
    d.Width = w; d.Height = h; d.MipLevels = 1; d.ArraySize = 1;
    d.Format = typeless ? DXGI_FORMAT_R8G8B8A8_TYPELESS : DXGI_FORMAT_R8G8B8A8_UNORM;
    d.SampleDesc.Count = msaa ? 4 : 1;
    d.Usage = D3D11_USAGE_DEFAULT;
    d.BindFlags = D3D11_BIND_RENDER_TARGET | (msaa ? 0 : D3D11_BIND_SHADER_RESOURCE);
    // --pattern: the top quarter of the texture is white, the left eighth green, the rest the eye's color
    std::vector<uint32_t> pixels;
    D3D11_SUBRESOURCE_DATA init{};
    if (pattern) {
      pixels.resize((size_t)w * h, eye == 0 ? 0xff2020c0u : 0xffc02020u); // ABGR in memory order RGBA
      for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = 0; x < w; ++x) {
          if (y < h / 4) pixels[(size_t)y * w + x] = 0xffffffffu;
          else if (x < w / 8) pixels[(size_t)y * w + x] = 0xff20c020u;
        }
      init.pSysMem = pixels.data();
      init.SysMemPitch = w * 4;
    }
    hr = dev->CreateTexture2D(&d, pattern ? &init : nullptr, &tex[eye]);
    if (FAILED(hr)) { say("CreateTexture2D failed 0x%08lx", (unsigned long)hr); return 1; }
    D3D11_RENDER_TARGET_VIEW_DESC rd{};
    rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    rd.ViewDimension = msaa ? D3D11_RTV_DIMENSION_TEXTURE2DMS : D3D11_RTV_DIMENSION_TEXTURE2D;
    hr = dev->CreateRenderTargetView(tex[eye], &rd, &rtv[eye]);
    if (FAILED(hr)) { say("CreateRenderTargetView failed 0x%08lx", (unsigned long)hr); return 1; }
  }

  vr::VRTextureBounds_t bounds = {0, 1, 1, 0}; // vMin > vMax: a vertically flipped image
  bool controllers_seen = false, input_seen = false;
  for (int frame = 0; frame < frames; ++frame) {
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::EVRCompositorError ce = comp->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
    if (frame < 3 || frame % 30 == 0)
      say("frame %d: WaitGetPoses=%d hmd valid=%d pos=(%.2f %.2f %.2f) ctl1 conn=%d valid=%d ctl2 conn=%d valid=%d", frame,
          (int)ce, (int)poses[0].bPoseIsValid, poses[0].mDeviceToAbsoluteTracking.m[0][3],
          poses[0].mDeviceToAbsoluteTracking.m[1][3], poses[0].mDeviceToAbsoluteTracking.m[2][3],
          (int)poses[1].bDeviceIsConnected, (int)poses[1].bPoseIsValid, (int)poses[2].bDeviceIsConnected,
          (int)poses[2].bPoseIsValid);
    const float c = (frame % 60) / 60.0f;
    const float left[4] = {c, 0.2f, 0.2f, 1}, right[4] = {0.2f, 0.2f, c, 1};
    if (!pattern) {
      ctx->ClearRenderTargetView(rtv[0], left);
      ctx->ClearRenderTargetView(rtv[1], right);
    }
    for (int eye = 0; eye < 2; ++eye) {
      vr::Texture_t vt = {tex[eye], vr::TextureType_DirectX, vr::ColorSpace_Auto};
      ce = comp->Submit((vr::EVREye)eye, &vt, flipped ? &bounds : nullptr, vr::Submit_Default);
      if (ce != vr::VRCompositorError_None || frame < 2) say("frame %d: Submit eye %d -> %d", frame, eye, (int)ce);
    }
    vr::VREvent_t ev;
    while (sys->PollNextEvent(&ev, sizeof(ev)))
      say("frame %d: event %u device %u", frame, ev.eventType, ev.trackedDeviceIndex);
    if (manifest) {
      vr::IVRInput* in = vr::VRInput();
      vr::VRActiveActionSet_t active{};
      active.ulActionSet = set;
      vr::EVRInputError ie = in->UpdateActionState(&active, sizeof(active), 1);
      vr::InputDigitalActionData_t fire{}, jump{};
      vr::InputAnalogActionData_t move{};
      vr::InputPoseActionData_t pose{};
      vr::EVRInputError e1 = in->GetDigitalActionData(a_fire, &fire, sizeof(fire), vr::k_ulInvalidInputValueHandle);
      vr::EVRInputError e2 = in->GetDigitalActionData(a_jump, &jump, sizeof(jump), vr::k_ulInvalidInputValueHandle);
      vr::EVRInputError e3 = in->GetAnalogActionData(a_move, &move, sizeof(move), vr::k_ulInvalidInputValueHandle);
      vr::EVRInputError e4 = in->GetPoseActionDataForNextFrame(a_pose, vr::TrackingUniverseStanding, &pose, sizeof(pose),
                                                              vr::k_ulInvalidInputValueHandle);
      static bool last_fire = false, last_jump = false;
      if (frame % 30 == 0 || fire.bState != last_fire || jump.bState != last_jump)
        say("frame %d: actions err %d/%d/%d/%d/%d fire active=%d state=%d jump active=%d state=%d move active=%d (%.2f %.2f) pose "
            "active=%d valid=%d pos=(%.2f %.2f %.2f)",
            frame, (int)ie, (int)e1, (int)e2, (int)e3, (int)e4, (int)fire.bActive, (int)fire.bState, (int)jump.bActive,
            (int)jump.bState, (int)move.bActive, move.x, move.y, (int)pose.bActive, (int)pose.pose.bPoseIsValid,
            pose.pose.mDeviceToAbsoluteTracking.m[0][3], pose.pose.mDeviceToAbsoluteTracking.m[1][3],
            pose.pose.mDeviceToAbsoluteTracking.m[2][3]);
      last_fire = fire.bState; last_jump = jump.bState;
      if (frame == 100)
        say("frame %d: haptic -> %d", frame,
            (int)in->TriggerHapticVibrationAction(a_haptic, 0, 0.1f, 160.0f, 0.8f, vr::k_ulInvalidInputValueHandle));
    }
    if (!manifest && frame >= 20) {
      for (uint32_t dev_index = 1; dev_index <= 2; ++dev_index) {
        vr::VRControllerState_t state{};
        const bool ok = sys->GetControllerState(dev_index, &state, sizeof(state));
        if (ok && !controllers_seen) {
          controllers_seen = true;
          char type[128] = {};
          sys->GetStringTrackedDeviceProperty(dev_index, vr::Prop_ControllerType_String, type, sizeof(type));
          say("frame %d: controller %u state ok, type '%s', class %d", frame, dev_index, type,
              (int)sys->GetTrackedDeviceClass(dev_index));
        }
        if (ok && !input_seen && (state.ulButtonPressed || state.rAxis[1].x > 0.01f || state.rAxis[0].x != 0)) {
          input_seen = true;
          say("frame %d: controller %u input: pressed=%llx trigger=%f stick=(%f %f)", frame, dev_index,
              (unsigned long long)state.ulButtonPressed, state.rAxis[1].x, state.rAxis[0].x, state.rAxis[0].y);
        }
      }
    }
  }
  say("controllers_seen=%d input_seen=%d input_available=%d", (int)controllers_seen, (int)input_seen,
      (int)sys->IsInputAvailable());
  say("step: VR_Shutdown");
  vr::VR_Shutdown();
  say("done");
  return 0;
}
