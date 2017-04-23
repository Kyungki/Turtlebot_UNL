#!/bin/bash

###
# Define definitions
###

. "$DABIT_DIR/Setup/automated/definitions.sh"

###
# Reset Files
###

confirm "Resetting Workspace. Are you sure? [y/N]"
if [[ "$?" -eq 0 ]]; then
  . "$_dabit_dir/Setup/automated/backup.sh" "mv"
  . "$_dabit_dir/Setup/automated/automate.sh"
fi


echo "RESET COMPLETE"
