#! /usr/bin/env python3

from forcedimension import HapticDevice
from forcedimension.containers import Vec3, PollGroup

def main():
    # Initial config:
    b = 5
    k = 150

    # Preallocate space for force request buffer
    force_req = Vec3()

    # Open a HapticDevice with context management
    with HapticDevice() as h:
        print(h.spec_str)

        def dynamics():
            force_req[:] = -k * h.pos - b * h.v
            h.req(force_req)
            h._config

            if h.get_button():
                h.regulator.notify_end()

        h.regulator.initialize() \
                   .start_drd() \
                   .regulate_pos(False) \
                   .poll(
                       *h.regulator.DEFAULT_REGULATOR_POLL_GROUPS,
                       PollGroup(
                           targets=[dynamics], wait_for=1e-3, name="DYNAMICS"
                       )
                    )

        # Sleep the main thread until the end event is set or an exception occurs in a polled thread
        h.regulator.wait_end()

if __name__ == "__main__":
    main()
