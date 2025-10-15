#!/bin/bash

# terminate on errors:
set -e

# Make sure remote refs are up-to-date
git fetch origin master -q

# Check if HEAD is ahead of origin/master
if [ -n "$(git rev-list origin/master..HEAD 2>/dev/null)" ]; then
    echo "Local commits found — pushing to origin/master..."
    git push origin HEAD:refs/heads/master
else
    echo "No new commits to push."
fi
