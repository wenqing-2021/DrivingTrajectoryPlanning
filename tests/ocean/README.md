# OCEAN tests

`test_ocean_math` covers the pieces that the planner depends on and that are easy
to get subtly wrong:

- the tangent majorant of `exp(d)` for `d <= 0`,
- the exact one-dimensional time block and its bounds,
- the exact partial minimization over the distance variable and its KKT condition,
- reverse-drive dynamics, the dual cone certificate of a rectangle pair,
- rigid-transform invariance of the certificate, convex decomposition of a
  concave obstacle with preserved area, explicit-zero parameter handling,
- OSQP reuse across changing problem dimensions, and the worker pool contract
  (each task once, exception propagation, timeout drain, reuse after a timeout).

`test_ocean_parameters` checks that an invalid `ocean_params` configuration is
reported as an explicit `invalid_parameters` failure instead of crashing or
planning with the bad setting, and that explicit zeros survive defaulting.

`test_ocean_cases_1` and `test_ocean_cases_4` run the 14-case benchmark harness on
`Case1` and `Case13` with one and four workers and two repeats each. They assert
success, the independent geometry/dynamics validation, and bitwise-identical
repeats through the public `solver.run` entry point.

Run everything with:

```bash
BUILD_TYPE=Release BUILD_JOBS=16 bash scripts/build.sh
uv run --frozen --no-sync .venv/bin/ctest --test-dir build/cmake --output-on-failure
```
