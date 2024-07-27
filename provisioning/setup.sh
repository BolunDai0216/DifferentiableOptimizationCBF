#!/bin/zsh
set -e

REQUIRED_DIRS=(
  "$HOME/diffoptcbf-devcontainer"
)

for dir in "${REQUIRED_DIRS[@]}"; do
  if [ ! -d "$dir" ]; then
    echo "Creating directory $dir"
    mkdir -p $dir
  fi
done

if [ ! -f "$HOME/diffoptcbf-devcontainer/.zsh_history" ]; then
  echo "Creating file $HOME/diffoptcbf-devcontainer/.zsh_history"
  touch $HOME/diffoptcbf-devcontainer/.zsh_history
fi