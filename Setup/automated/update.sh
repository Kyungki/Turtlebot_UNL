#!/bin/bash
answer=confirm "Moving all working files to ~/old_workspace. Are you sure? [y/N]"
if [ answer -eq 1]; then
  echo "True"
fi
