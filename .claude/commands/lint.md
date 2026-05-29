Run the Flatland CI pre-build lint gate locally and fix what it flags.

This is the same check CI runs (`scripts/ci_prebuild.sh`): clang-format `--style=file` over all
tracked `.c/.h/.cpp` files except `thirdparty/`, plus clang-tidy.

1. Check formatting (read-only, mirrors CI):
   ```bash
   git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" \
     | xargs clang-format --style=file -output-replacements-xml | grep -c "<replacement "
   ```
   A non-zero count = files need reformatting.
2. Auto-fix formatting in place:
   ```bash
   git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" \
     | xargs clang-format --style=file -i
   git diff --stat
   ```
3. Review the diff — only formatting should change. Never touch files under `thirdparty/`.
4. For clang-tidy findings, respect suppressions in `scripts/clang_tidy_ignore.yaml`
   (`scripts/parse_clang_tidy.py` parses the output).

Report what was reformatted. Do not commit unless asked.
