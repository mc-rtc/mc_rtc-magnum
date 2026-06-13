{
  description = "mc-rtc-magnum standalone gui visualization";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
  inputs.mc-rtc.url = "github:jrl-umi3218/mc_rtc/pull/525/head";

  inputs.mc-rtc-imgui.url = "github:mc-rtc/mc_rtc-imgui/nix";
  inputs.mc-rtc-imgui.flake = false;

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        # { this section allows to build mc-rtc-magnum against multiple versions of fmt
        #   using nix build -L ".#pkgs-fmt_<version>"

        extends.fmt_9 = final: _prev: { fmt = final.fmt_9; };
        extends.fmt_10 = final: _prev: { fmt = final.fmt_10; };
        extends.fmt_11 = final: _prev: { fmt = final.fmt_11; };
        extends.fmt_12 = final: _prev: { fmt = final.fmt_12; };

        # FIXME: building with pkgs-fmt_10 causes qt6base to rebuild
        # even though nothing here depends on it.
        # This is due to flakoboros' ros overlay here
        # https://github.com/Gepetto/flakoboros/blob/main/lib/mk-lib.nix#L95
        overlays = [
          (_final: _: {
            qt6.qtbase = null;
            qt6.wrapQtAppsHook = null;
          })
        ];
        # } end fmt version extensions

        # temporarely override mc_rtc to test pr #525
        overrideAttrs.mc-rtc = {
          src = inputs.mc-rtc;
        };

        overrideAttrs.mc-rtc-imgui = {
          src = inputs.mc-rtc-imgui;
        };

        overrideAttrs.mc-rtc-magnum = {
          src = lib.cleanSource ./.;
        };
      }
    );
}
