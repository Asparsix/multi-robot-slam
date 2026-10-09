# Multi-Robot House Mission — Hackathon Overview

**Plain language.** No heavy math. For internal demos / hackathon.

Full versions: [TECHNICAL_OVERVIEW.tex](TECHNICAL_OVERVIEW.tex) · [TECHNICAL_OVERVIEW.txt](TECHNICAL_OVERVIEW.txt)

---

## The problem

Four robots, one house, ten places to visit. Split the work, don’t crash into walls or each other, finish every place.

## Five steps

1. **Multi-robot SLAM** — Build one shared map (many position guesses / “particles,” laser + motion glued into a floor plan).
2. **Localization** — Keep track of where each robot is on that map.
3. **CBBA** — Auction tasks into ordered to-do lists. Bid on *extra travel* if you add a task to your route (marginal effort). Does **not** drive robots.
4. **MAPF** — Plan timed paths so robots don’t sit on the same spot at the same time (we use simple priority planning, not CBS/ECBS).
5. **Execution** — Drive beat-by-beat together so the timing from MAPF is kept.

Loop steps 4–5 for each next task in the lists until everything is done.

## One-liner

**Map → know positions → split jobs → plan traffic → drive.**
