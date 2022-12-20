#!/bin/bash -e

function main {
	local tempFile=$(mktemp)

	# work over each commit and append all files in tree to $tempFile
	local IFS=$'\n'
	local commitSHA1
	for commitSHA1 in $(git rev-list --all); do
		git ls-tree -r --long "$commitSHA1" >>"$tempFile"
	done

	# sort files by SHA-1, de-dupe list and finally re-sort by filesize
	sort --key 3 "$tempFile" | \
		uniq | \
		sort --key 4 --numeric-sort --reverse

	# remove temp file
	rm "$tempFile"
}


main
