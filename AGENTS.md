# ChirpStack Concentratord

LoRa(WAN) concentrator daemon forked from [ChirpStack](https://github.com/chirpstack/chirpstack-concentratord). Abstracts gateway hardware (SX1301, SX1302, 2.4GHz) behind a ZeroMQ API, allowing multiple packet-forwarding applications to run simultaneously.

- **Language:** Rust (edition 2024, toolchain 1.89.0)
- **License:** MIT
- **Origin:** Fork of `brocaar/chirpstack-concentratord` hosted at `github.com/yosensi/chirpstack-concentratord`
- **Development branch:** `master`

## Project Structure

```
chirpstack-concentratord/
  chirpstack-concentratord-sx1301/   # Binary crate for SX1301 concentrators
  chirpstack-concentratord-sx1302/   # Binary crate for SX1302 concentrators (used by EverLink PRO)
  chirpstack-concentratord-2g4/      # Binary crate for 2.4 GHz concentrators
  gateway-id/                        # Helper: reads gateway EUI
  gateway-location/                  # Helper: reads GNSS location
  libconcentratord/                  # Shared library (ZMQ API, JIT queue, GNSS, config)
  libloragw-sx1301/                  # Rust FFI bindings to SX1301 HAL (C)
  libloragw-sx1302/                  # Rust FFI bindings to SX1302 HAL (C)
  libloragw-2g4/                     # Rust FFI bindings to 2.4 GHz HAL (C)
  cross/                             # Dockerfiles for cross-rs cross-compilation
  packaging/                         # Vendor-specific packaging scripts (Kerlink, Multitech)
```

## EverLink PRO Relevance

The EverLink PRO gateway uses the **SX1302** variant (`chirpstack-concentratord-sx1302`). The binary is cross-compiled for ARMv7 and installed via the Yocto recipe in `meta-yosensi`.

On the gateway:
- Config: `/etc/chirpstack-concentratord/`
- Default configs: `/etc/default/chirpstack-concentratord/sx1302/`
- Systemd service: `chirpstack-concentratord.service`

## Building

Requires Docker (for [cross-rs](https://github.com/cross-rs/cross)) and optionally Nix for the dev shell.

```bash
# Install cross-rs (one-time)
make dev-dependencies

# Build for all targets
make build

# Build ARMv7 only (for EverLink PRO gateway)
make build-armv7-unknown-linux-musleabihf

# Build + distributable packages
make dist

# Run tests (clippy + test suite via cross)
make test

# Clean
make clean
```

## Cross-Compilation Targets

| Target                              | Use case              |
|-------------------------------------|-----------------------|
| `armv7-unknown-linux-musleabihf`    | EverLink PRO (STM32MP) |
| `aarch64-unknown-linux-musl`        | ARM64 gateways        |
| `armv5te-unknown-linux-musleabi`    | Multitech Conduit     |
| `x86_64-unknown-linux-musl`        | Testing / CI          |

## Testing

```bash
make test
```

Runs `cross clippy` and `cross test` against `x86_64-unknown-linux-musl`.

## Configuration

Each concentrator variant has a `config/` directory with TOML configuration templates:
- `concentratord.toml` - main daemon config (ZMQ endpoints, logging, GPS)
- `region_*.toml` - region-specific radio parameters
- `channels_*.toml` - channel plan definitions

## Dependencies in Workspace

- **depended_by:** `meta-yosensi` (Yocto recipe installs the pre-built binary)
- The binary is **not** built by Yocto itself - it is pre-compiled and placed in `meta-yosensi/...recipes-chirpstack/.../files/`
