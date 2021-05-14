# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Open Issues
- Rename this to ROS Script Wrapper (since technically any script can be executed/started/stopped).
- Add recording scripts.

## [UNRELEASED]
### Added
- Created initial repository with AAU_CNS License and Readme
- Added ROS service interface to start/stop (record) script execution via `/data_recorder/record`
  This uses the `std_srvs/SetBool` to start/stop the execution and returns `true` if successful.
- Added sample launch file and record script.

[Unreleased]: https://gitlab.aau.at/aau-cns/ros_pkgs/data_recorder/-/compare/develop...main
