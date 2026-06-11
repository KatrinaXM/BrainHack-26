# BrainHack-26 RoboVerse

Team's code for the **BrainHack-26 RoboVerse** autonomous-drone competition.
- **Qualifier:** passed.
- **Finals:** Pre-University category, Challenge 2 only (HULA swarm landing + ambush).

---

## 👉 Start here

**[`START_HERE.md`](START_HERE.md)** — single comprehensive guide. Read this if you've just joined the team or are picking the project up after a break. Covers status, how to run the test mission, what to do on competition day, troubleshooting, and a glossary.

---

## Quick-launch (for people who already know the project)

```bash
cd ~/BrainHack-26/codes/finals
./run_stage2.sh --short     # 25-second end-to-end smoke test (mock drones)
```

Run the test suite:
```bash
cd ~/BrainHack-26/codes/finals
python3 -m unittest tests.test_stage2 -v
```

Real hardware mode:
```bash
./run_stage2.sh --real --pads <organizers' pad file>
```

---

## Repo layout (1-minute version)

| Path | What's in it |
|---|---|
| `codes/finals/` | **Active stack.** Pre-U Stage 2 orchestrator, mock, tests, launcher. |
| `materials/` | Competition rules + workshop slides (PDFs). |
| `references/finalist_codes/` | Workshop reference code for Finals (mostly Stage 1 / University). |
| `references/qualifier_codes/` | Workshop reference for the Qualifier. |
| `codes/` (top level) | Qualifier-era code. Kept for reference; not used in Finals. |
| `scripts/`, `px4-patches/`, `sdf-patches/` | Qualifier simulator helpers and patches. Not used for Finals. |
| `TUTORIAL.md` | Long-form concept reference. Chapter 24 (pyhulax) is the Pre-U-relevant part. |
| `RUNBOOK.md` | Phased prep checklist. |
| `docs/` | Deeper dives (currently: a Stage 1 reference dissection). |

---

For everything else, read [`START_HERE.md`](START_HERE.md).
