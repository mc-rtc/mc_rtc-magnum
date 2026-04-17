{
  description = "mc-rtc-magnum standalone gui visualization";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
  inputs.mc-rtc-magnum-standalone.url = "github:mc-rtc/mc_rtc-magnum/nix";

  inputs.mc-rtc-imgui.url = "github:mc-rtc/mc_rtc-imgui/nix";
  inputs.mc-rtc-imgui.flake = false;

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { ... }:
      {
        overrideAttrs.mc-rtc-imgui = {
          src = inputs.mc-rtc-imgui;
        };

        overrideAttrs.mc-rtc-magnum = {
          src = builtins.warn "warning: mc-rtc-magnum on main branch does not support nix, running from the latest locked commit in nix branch" inputs.mc-rtc-magnum-standalone;
        };
      }
    );
}
