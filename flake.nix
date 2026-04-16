{
  description = "mc-rtc-magnum standalone gui visualization";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.mc-rtc-magnum = {
          src = lib.cleanSource ./.;
        };
      }
    );
}
