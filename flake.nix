{
  description = "mc-rtc-magnum standalone gui visualization";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";

  inputs.mc-rtc-imgui.url = "github:mc-rtc/mc_rtc-imgui/nix";
  inputs.mc-rtc-imgui.flake = false;

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.mc-rtc-imgui = {
          src = inputs.mc-rtc-imgui;
        };

        overrideAttrs.mc-rtc-magnum = {
          src = lib.cleanSource ./.;
        };
      }
    );
}
