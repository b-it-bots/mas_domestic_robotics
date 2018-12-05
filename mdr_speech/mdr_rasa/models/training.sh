#!/bin/bash

# set CWD to the script's directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# check number of arguments
if [ "$#" -ne 1 ]; then
    echo "Format: ./training.sh <model_name>"
    exit 1
fi

# train the model
python -m rasa_nlu.train -c "$1/config.yml" --data "$1/training_data.md" -o "$1" --project "generated" --fixed_model_name "model" --verbose
