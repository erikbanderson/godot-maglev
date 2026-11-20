#!/bin/sh
printf '\033c\033]0;%s\a' Maglev
base_path="$(dirname "$(realpath "$0")")"
"$base_path/test1-4.x86_64" "$@"
