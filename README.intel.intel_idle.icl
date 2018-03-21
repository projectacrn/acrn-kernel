intel_idle and turbostat enabling for Icelake

Much of this was coded "to spec", with *no* bench testing (hardware is severely
limited). Therefore, expect to find bugs.

Use reference values lifted from Icelake Core and Uncore BIOS Specification
(BWG). These values are not final, nor are they optimized.

Also make turbostat aware of Icelake to ease debug and analysis.
