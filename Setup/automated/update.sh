#!/bin/bash
confirm "Moving all working files to ~/old_workspace. Are you sure? [y/N]"
if [[ "$?" -eq 0 ]]; then
  echo "True"
fi
