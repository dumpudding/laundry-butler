SRC="/home/laundrybutler/Aloha Shared SSD/dxx_data/0727_300eps"
DEST="/raid/dgxtest/laundry-butler/dxx_data/usable_episode"
LIST="/tmp/laundry_butler_usable_episodes.nul"

rsync \
  -ar \
  --partial \
  --append-verify \
  --from0 \
  --files-from="$LIST" \
  --info=progress2,stats2 \
  "$SRC/" \
  "dgx:${DEST}/"