workspace := justfile_directory()

# Start the devcontainer
devcontainer-up:
    devcontainer up --workspace-folder {{workspace}}

# Open a shell in the running devcontainer
devcontainer-exec:
    devcontainer exec --workspace-folder {{workspace}} bash

# Build the devcontainer image
devcontainer-build:
    devcontainer build --workspace-folder {{workspace}}
