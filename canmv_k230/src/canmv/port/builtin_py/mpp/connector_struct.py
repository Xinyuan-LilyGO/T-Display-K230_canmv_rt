import uctypes
from mpp import connector_def

HX8377_V2_MIPI_4LAN_1080X1920_30FPS = const(0)
LT9611_MIPI_4LAN_1920X1080_30FPS = const(1)
ST7701_V1_MIPI_2LAN_480X800_30FPS = const(2)
ST7701_V1_MIPI_2LAN_480X854_30FPS = const(3)
ILI9806_MIPI_2LAN_480X800_30FPS = const(4)

LT9611_MIPI_4LAN_1920X1080_60FPS = const(101)
LT9611_MIPI_4LAN_1280X720_60FPS = const(110)
LT9611_MIPI_4LAN_1280X720_50FPS = const(111)
LT9611_MIPI_4LAN_1280X720_30FPS = const(112)
LT9611_MIPI_4LAN_640X480_60FPS = const(120)
RM69A10_MIPI_2LAN_568X1232_60FPS=const(140)
VIRTUAL_DISPLAY_DEVICE = const(200)

def k_connectori_phy_attr(**kwargs):
    layout = uctypes.NATIVE
    buf = bytearray(uctypes.sizeof(connector_def.k_connectori_phy_attr_desc, layout))
    s = uctypes.struct(uctypes.addressof(buf), connector_def.k_connectori_phy_attr_desc, layout)
    connector_def.k_connectori_phy_attr_parse(s, kwargs)
    return s

def k_connector_info(**kwargs):
    layout = uctypes.NATIVE
    buf = bytearray(uctypes.sizeof(connector_def.k_connector_info_desc, layout))
    s = uctypes.struct(uctypes.addressof(buf), connector_def.k_connector_info_desc, layout)
    connector_def.k_connector_info_parse(s, kwargs)
    return s
