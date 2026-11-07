# Host compile-check stubs

Minimal stand-ins for XTC system headers (`xcore/*.h`, `platform.h`,
`xassert.h`) and for the FreeRTOS / fwk_rtos / device-control headers, so the
four-mic pipeline sources can be type-checked on a host compiler against the
**real** fwk_voice v0.9.0, lib_xcore_math and Satellite1 application headers.

They declare only what the pipeline sources use, with signatures copied from
the real headers. They are never linked into firmware. A clean compile here is
not a substitute for an XTC build: it cannot check xcore-specific attributes,
stack-size annotations, memory placement or the `.xn` platform.
