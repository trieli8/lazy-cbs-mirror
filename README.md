# Lazy CBS: A multi-agent pathfinding solver, using a lazy clause generation backend

## Usage
Build one variant per build directory so you can run experiments side by side:

```bash
./build_variants.sh
```

That creates:

* `build-og/og`
* `build-target/target`
* `build-confilct/confilct`

Each binary still accepts `--no-target-symmetry` and `--no-conflict-tiebreaker` if you want to override the baked-in defaults for a specific run.

If you only want the default build, use:

```bash
mkdir build && cd build
cmake ..
make
```

Then run `./lazycbs_mapf -i ../example/input.yaml` from that build folder.
