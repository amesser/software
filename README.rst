Fieldbus Analyzer
=======================

I started messing around with the CAN bus in my circuits. For that
reason I needed a simple CAN tracer. I found an old Hilscher 
NXSB100 in my box. It ha a DeviceNet interface which can be modified to
be able to receive standard CAN as well. I hooked up a quick & dirty 
solution to dump all can frames to the ethernet interface.

To rebuild the solution, you would need the Standard Ethernet Mac HAL
and CAN HAL from Hilscher. They're usually shipped with the Hilscher 
Boards if you buy one. Copy the files into a flat subfolder hirachy
within ``3rdparty`` subfolder. See the contained wscript for details.

The software will also work with other Hilscher boards. It might be 
required to adjust the CAN channel.

Licenses
--------
Everything located under ``firmware`` subfolder is licensed under 
the GPL.

Besides that the external modules located in ``externals``
subdirectory ship with their own licensing conditions.

Author
------

Andreas Messer <andi@bastelmap.de>

