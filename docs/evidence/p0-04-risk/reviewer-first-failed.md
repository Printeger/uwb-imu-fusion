# P0-04 first independent review — FAILED

Reviewer role: read-only independent reviewer; no worktree modifications.

Verdict: **FAILED**. The step was not eligible for the master-table update,
commit, or golden tag.

## Blocking findings

1. `risk_budget_audit.cpp:160-185,294-313` converted quad-precision
   intermediates to binary64 and rebuilt the total from rounded terms. For
   `allocation=5.226175689748075e-15`,
   `prior=2.0503228027277203e-12`, `beta=0.005889712081465759`, and
   `budget=1.2075810982130189e-14`, the production terms rounded to the budget
   although the authoritative `prior*beta` exceeded it by
   `7.605739429735566e-31`.
2. `fde_manager.cpp:211-214` set `selected=true` and `selected_action` before
   the complete risk gate; a later UNKNOWN/over-budget result only prohibited
   commit.
3. `fde_manager.cpp:233-269` constructed each action's supposed common
   reference from that same action (`L_ref=L_action`, `p_ref=p_action`, shared
   flags and triangle proof true). The zero self-transfer and hash collisions
   could therefore create false shared events.
4. The final `integrity_monitor` consumer copied the pre-selection PL
   `risk_budget_valid`; the complete ledger remained diagnostic rather than
   authoritative.
5. The six-argument `FdeManager::decide` ABI symbol and public layout
   compatibility were lost by the initial change.

## Positive evidence that did not clear the blockers

- Clean build and complete CTest: 27/27 PASS.
- Retained hashes were internally consistent.

## Required repair

Use authoritative quad accumulation with upward export, run the risk gate
before publishing a selected action, require a real common reference or charge
singletons, make the final ledger authoritative, and preserve the golden ABI.
