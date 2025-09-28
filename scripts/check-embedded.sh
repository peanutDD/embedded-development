#!/bin/sh
set -eu

tmp_pkgs="$(mktemp)"

# Discover embedded packages that depend on stm32f4xx-hal
find . -name Cargo.toml | while read f; do
  if grep -q 'stm32f4xx-hal' "$f"; then
    name="$(sed -n 's/^name[[:space:]]*=[[:space:]]*"\([^" ]*\)"/\1/p' "$f" | head -n1)"
    if [ -n "$name" ]; then
      echo "$name"
    fi
  fi
done | sort -u > "$tmp_pkgs"

count="$(wc -l < "$tmp_pkgs" | tr -d ' ')"
echo "Found $count embedded packages using stm32f4xx-hal"
echo

failures="$(mktemp)"

while read p; do
  echo "Checking $p (thumbv7em-none-eabihf)"
  if ! cargo check -p "$p" --target thumbv7em-none-eabihf; then
    echo "FAILED: $p" | tee -a "$failures"
  fi
  echo
done < "$tmp_pkgs"

fail_count="$(wc -l < "$failures" | tr -d ' ')"
echo "Summary: $fail_count failures out of $count packages"
if [ "$fail_count" -gt 0 ]; then
  echo "Failures:"
  sed 's/^/- /' "$failures"
fi

rm -f "$tmp_pkgs" "$failures"