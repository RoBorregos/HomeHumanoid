#!/bin/sh
# Downloads for the Dockerfiles that survive a flaky network: retries, and
# fetches only the pinned commit (no history).
#   fetch.sh git URL COMMIT DIR     shallow checkout of COMMIT into DIR
#   fetch.sh tar URL DIR            download a .tar.gz/.tgz and extract it into DIR
set -e
retry() {
  n=1
  until "$@"; do
    [ $n -ge 5 ] && { echo "failed after $n tries: $*" >&2; return 1; }
    echo "retry $n: $*" >&2; n=$((n + 1)); sleep $((n * 5))
  done
}
case "$1" in
  git)
    rm -rf "$4"; git init -q "$4"; git -C "$4" remote add origin "$2"
    retry git -C "$4" fetch -q --depth 1 origin "$3"
    git -C "$4" checkout -q FETCH_HEAD
    retry git -C "$4" submodule update -q --init --recursive --depth 1 ;;
  tar)
    mkdir -p "$3"
    retry sh -c "curl -fsSL --connect-timeout 30 '$2' -o /tmp/fetch.tgz"
    tar xzf /tmp/fetch.tgz -C "$3"; rm -f /tmp/fetch.tgz ;;
  *) echo "usage: fetch.sh git URL COMMIT DIR | tar URL DIR" >&2; exit 2 ;;
esac
