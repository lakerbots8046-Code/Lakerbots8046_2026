# FeedFromCenter Hood Offset Implementation

## Plan Steps
- [x] User approved plan to add +2 hood offset constant for FeedFromCenter only
- [ ] Add `kFeedHoodOffsetRotations = 2.0` to Constants.java (FeedFromCenter class)
- [ ] Update `interpolateHoodPosition()` in FeedFromCenterCommand.java to add offset unconditionally
- [ ] Verify changes (read files, check compilation if possible)
- [ ] Test: Dashboard shows +2 rot across distances; no table changes
- [ ] Complete task
