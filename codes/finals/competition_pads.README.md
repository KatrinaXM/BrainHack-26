# competition_pads.json — provenance & TODO

**Source:** organizer announcement, 2026-06-10. These are the **Stage 2
landing points** (no ArUco markers on the landing points themselves — those
are only on the ground robots). Coordinates are in **metres**; `z=0` (ground).
Three pads in a horizontal row at y=5.5; IDs ascend left→right (confirmed with
the team, 2026-06-10).

| id  | x (m) | y (m) | position   |
|-----|-------|-------|------------|
| 7   | 2.5   | 5.5   | left side  |
| 10  | 5.5   | 5.5   | middle     |
| 12  | 7.5   | 5.5   | right side |

> Earlier placeholder (5 pads: 11/45/51/67/101) is superseded by the above.

## ⚠ BEFORE YOU FLY — availability flags

Each pad carries an **availability flag** (`"available": true|false`; the older
`"valid"` key still works as a synonym). The mission **only ever assigns
AVAILABLE pads** to drones, and Pre-U lands **3 HULAs**, so **at least 3 pads
must be available**. `select_pads()` picks the first 3 available pads in file
order — currently plane1→7, plane2→10, plane3→12.

The Finals defines up to **5 landing pads, some of which may be unavailable**.
Add the other pads and flag availability as the organizers announce it.

### Easiest way: the `configure.py` flag-setter (no hand-editing JSON)

```bash
python configure.py show                       # list pads + availability + drones
python configure.py pad 7  --x 2.5 --y 5.5     # add/update a pad's coordinates
python configure.py pad 11 --unavailable       # flag a pad NOT available
python configure.py pad 11 --available         # flag it available again
python configure.py check                      # verify >=3 are available
```

Or edit `competition_pads.json` by hand: set `"available": false` on any pad to
exclude it (keep >=3 available), and reorder the file to control which available
pads are used / which drone gets which pad.

## Drones (separate config)

Drone **IPs** can be auto-discovered (`dola`, UDP 8668) or set explicitly with
`configure.py drone 1 --ip 192.168.1.101` (writes `drones.json`) or
`BH26_HULA_IPS`. Drone **arena positions are NOT discoverable** — the HULA only
broadcasts its IP/identity and self-positions by optical flow; for a shared
absolute frame you'd need the QR-localization mat (`BH26_NAV_MODE=move_to` +
`BH26_QR_LOCALIZATION=1`).

## Marker IDs (separate thing — do not confuse with pads)

The ArUco markers we DETECT are on the **ground robots**, dictionary
`DICT_7X7_1000`, IDs `11, 45, 51, 67, 101`. That is wired into the detector
default (`BH26_ARUCO_DICT`) and the expected-ID report (`BH26_EXPECTED_IDS`)
in `stage2_mission.py` — nothing to edit there unless the organizers change it.
