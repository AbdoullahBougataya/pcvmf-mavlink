"""Optional adapters are loaded only when their backend is selected."""


def create(options):
    if options.backend == "pymavlink":
        from .pymavlink import PymavlinkBackend

        return PymavlinkBackend(options)
    if options.backend == "mavsdk":
        from .mavsdk import MavsdkBackend

        return MavsdkBackend(options)
    from .fake import FakeBackend

    return FakeBackend(options)
