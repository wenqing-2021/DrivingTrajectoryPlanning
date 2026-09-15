# RDA workspace reuse and CPU thread parallelism

RDA separates fixed sparse structure from changing numerical values, and solves
the independent obstacle SOCPs on a persistent CPU thread pool. The shipped
configuration uses **16 worker threads**; `rda_params.workers: 1` selects the
serial implementation. No GPU is used. A process-based pool was also implemented
and measured during development; it matched the thread pool to within about 0.5%
at equal worker counts while using more memory and requiring shared-memory
marshalling, so it was removed in favour of the thread pool.

## Lifecycle

`solver::Solver` owns an `RDAWorkspace` and passes it through each newly created
`TrajPlanner` to `RDASolver`. Its structure key is the trajectory segment count
`T` and the ordered vector of obstacle edge counts. This also fixes the obstacle
count `M`. A changed key rebuilds the workspace and stops the old workers.
Matching keys reuse buffers across requests, while all problem values and ADMM
variables are refreshed. There is no obstacle pruning or dummy-obstacle padding.
Because the workspace is shared across requests, calls to `solver::Solver::Run`
for one solver instance must stay serialized; the binding already behaves that way.

`SparseValues` constructs compressed sparse matrices and coefficient offsets
once. Subsequent rounds accumulate values directly into the existing arrays,
including entries whose values cross zero. Duplicate coefficients are summed.
Static obstacle halfspaces are stored once rather than copied for every state.

The SU QP holds one OSQP instance throughout the ADMM loop and updates its data.
At the beginning of each new planning request, OSQP is reinitialized to clear
adaptive rho and scaling inherited from the preceding request. Caller-owned
matrix buffers remain allocated. This boundary is necessary: retaining OSQP
state across requests caused a repeated Case 8 request to collide. The repeated
request test protects this behavior. Warm starting the QP iterates is disabled
in the reusable path; tolerances and iteration limits are unchanged.

Each obstacle has one `ConeWorkspace` that keeps its reusable coefficient and
cone-layout buffers. Each ADMM round constructs a fresh `EiCOS::Solver` from those
buffers and solves it; the vendored EiCOS translation unit is used unchanged.
Numerical factorizations are still performed as required. Solver internals can
still allocate scratch memory; this is not a guarantee of zero heap allocations
throughout planning.

## Parallel round

1. The main thread solves SU and fills all obstacle SOCP values.
2. A persistent `ConeThreadPool` runs the assigned obstacles and the main thread
   waits for every worker to acknowledge the round.
3. Results are read back in obstacle order, then ADMM multipliers, residuals and
   collision checks are updated on the main thread.

Work distribution balances stored matrix nonzero counts. The effective worker
count is at most `M`. Workers own disjoint `ConeWorkspace` objects, so no solver
state is shared and no locking is needed around the solves. The pool creates its
threads once per structure, and each thread calls its own EiCOS instances through
the same in-memory workspace, so a round needs no marshalling, no extra copy and
no per-round startup. `Eigen::initParallel()` and `Eigen::setNbThreads(1)` are
called once on the owning thread before the workers start, because Eigen
documents the former for multi-threaded callers and the latter writes an
unsynchronized global. This build does not enable OpenMP.

`worker_timeout_ms` defaults to 5000 and applies to waiting for a parallel round,
not the full planning request. A running solve cannot be preempted from another
thread, so on timeout the pool lets the round drain before reporting failure and
the planning request fails; the workspace is therefore never left being written
by a worker. The pool is stopped and joined on structure changes and destruction,
and a partially completed round is never published. Numerical SOCP termination
handling is unchanged from the previous implementation.

## Reproduction

```bash
BUILD_TYPE=Release BUILD_JOBS=16 bash scripts/build.sh
UV_CACHE_DIR=$PWD/.cache/uv uv run --frozen --no-sync ctest --test-dir build/cmake --output-on-failure
UV_CACHE_DIR=$PWD/.cache/uv uv run --frozen --no-sync python tests/rda/benchmark_runtime.py \
  --workers 1 --repeats 2 --output solve_results/rda_runtime/serial.json
# No --workers or --workers 0 uses the configured 16 threads.
UV_CACHE_DIR=$PWD/.cache/uv uv run --frozen --no-sync python tests/rda/benchmark_runtime.py \
  --repeats 2 --output solve_results/rda_runtime/threads16.json
```

The benchmark measures the complete binding call, including per-request setup
and first-round worker startup. It stores states, reconstructed controls,
timestamps, per-axis endpoint errors, discrete footprint collisions, the maximum
absolute x/y Euler dynamics residual, and repeated-request state differences.
`--workers 0` keeps the configured worker count and a positive value overrides
it; the recorded `workers` field reports the effective setting. Identical requests
must agree within `1e-8`; collision or solve failure produces a nonzero exit
status. Collision validation checks full vehicle footprints at returned states;
it is not a continuous swept-volume certificate.

The `RDA runtime` log separates preparation, SU assembly, SU solve/update, cone
assembly, and cone solve/synchronization time. Figures and rendering are excluded.
Run timing experiments without concurrent builds or other benchmark jobs.

## Measurements (2026-09-15)

Release build, current 14 parking cases, 16 logical CPUs visible. A is the
original implementation at parent HEAD `242d47f`; B is serial workspace reuse with
the request-boundary OSQP reset; C adds the thread pool. Entries are seconds
(median over the recorded samples). A was measured once per case; the others run
per case as noted. These sample counts do not establish stable tail percentiles.

| Case | A (original) | B serial | C threads x8 | C threads x16 |
|---|---:|---:|---:|---:|
| 1 | 1.087 | 0.945 | 0.444 | 0.446 |
| 2 | 2.465 | 2.412 | 1.269 | 1.263 |
| 3 | 2.163 | 1.821 | 0.782 | 0.785 |
| 4 | 9.571 | 8.936 | 2.706 | 2.434 |
| 5 | 12.240 | 11.036 | 3.657 | 3.380 |
| 6 | 12.436 | 11.435 | 3.226 | 2.839 |
| 8 | 1.999 | 1.931 | 1.017 | 1.025 |
| 11 | 4.640 | 4.371 | 1.735 | 1.737 |
| 12 | 4.020 | 3.805 | 1.475 | 1.474 |
| 13 | 2.241 | 2.165 | 0.822 | 0.810 |
| 14 | 3.108 | 3.022 | 1.209 | 1.210 |
| 16 | 4.687 | 4.664 | 1.701 | 1.447 |
| 17 | 3.884 | 3.626 | 1.182 | 0.993 |
| 18 | 3.025 | 2.853 | 0.975 | 0.765 |
| **Sum** | **67.566** | **63.023** | **22.200** | **20.607** |

Peak sampled process-tree RSS for the full suite (one run each, includes the
Python benchmark process): serial 105 MiB, threads x8 110 MiB, threads x16
114 MiB. The pool reuses one address space, so worker count barely changes the
memory footprint.

Scaling behaves as expected: on the three heaviest cases (4/5/6, median of three)
threads x4 took 12.417 s, threads x8 9.562 s and threads x16 8.618 s. The larger
worker count is the better choice on this 16-core host, so the shipped default is
16 worker threads; the optimum tracks available cores up to the obstacle count.

## Verification

The default configuration (16 threads) reduced the aggregate of the 14 per-case
medians from **67.566 s** for A to **20.615 s**, a **3.3x** speedup, with Case 5 at
3.327 s and Case 18 at 0.772 s. 12/12 CTest tests pass. Every recorded run (serial,
threads x8 and x16, and the final default rerun) solved all 14 cases with zero
discrete collisions and a zero repeated-request state delta. Across worker counts
the returned states, controls and timestamps were **bitwise identical**: the pool
changes only the timing, not the solution.

Material limitation inherited from B: OSQP data reuse changes numerical
trajectories relative to A. Across cases, maximum absolute terminal x/y/heading
errors changed from `0.05357 m / 0.05897 m / 0.08004 rad` to
`0.05305 m / 0.05716 m / 0.08002 rad`. Configured terminal bounds remain
`0.05 m / 0.08 rad`; the small violations are numerical, not relaxed limits. The
worst x/y Euler dynamics residual increased from `0.01730 m` to `0.02809 m`
(Case 11), so workspace reuse is not claimed to be numerically identical to A or
to improve every trajectory-quality metric. Exported reconstructed steering can
exceed the configured front-wheel limit in both A and B; its maximum changed from
`1.15403` to `1.15076 rad`. This measurement does not certify all reconstructed
controls as constraint-feasible. The configured front/back steering limit
remains `0.75 rad` and the RS collision sampling parameter remains `0.1 m`.

Raw results, logs, per-case min/median/max times and an aggregate JSON summary are
retained under `solve_results/rda_runtime_20260915/` (ignored generated artifacts).
