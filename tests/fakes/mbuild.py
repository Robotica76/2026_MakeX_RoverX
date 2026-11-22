"""mbuild falso para pruebas en PC (ver world.py)."""
import world


class _QuadRGB:
    def get_offset_track(self, index=1):
        return world.sensors["offset"]()

    def get_line_sta(self, ch="all", index=1):
        return world.sensors["line_sta"]()

    def is_line(self, ch, index=1):
        return 0

    def get_color_sta(self, ch, index=1):
        return world.sensors["color"](ch)

    def get_red(self, ch, index=1):
        return world.sensors["rgb"](ch)[0]

    def get_green(self, ch, index=1):
        return world.sensors["rgb"](ch)[1]

    def get_blue(self, ch, index=1):
        return world.sensors["rgb"](ch)[2]

    def set_led(self, color="white", index=1):
        pass

    def off_led(self, index=1):
        pass


class _Ultrasonic2:
    def get(self, index=1):
        return world.sensors["distance"]()


quad_rgb_sensor = _QuadRGB()
ultrasonic2 = _Ultrasonic2()
