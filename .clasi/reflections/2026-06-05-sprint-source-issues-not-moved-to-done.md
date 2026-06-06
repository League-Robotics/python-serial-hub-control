---
date: 2026-06-05
sprint: 001 (closed)
category: emergent-gap
---

# Sprint source issues left `pending` after the delivering sprint closed

## What Happened

Sprint 001 built and delivered the new idiomatic `rhsp` library. Its two
source issues —
[rhsp-idiomatic-rewrite.md](../issues/done/rhsp-idiomatic-rewrite.md) and
[rhsp-build-a-new-idiomatic-python-library.md](../issues/done/rhsp-build-a-new-idiomatic-python-library.md)
— remained `status: pending` in the `.clasi/issues/` pool after the sprint was
closed. There was no `.clasi/issues/done/` directory at all; the archived sprint
`sprints/done/001-build-new-idiomatic-rhsp-library/` had **no `issues/`
subdirectory**; and CLASI status reported `assigned_to_sprint: 0`. The
stakeholder has hit this repeatedly ("I can't keep doing this").

The orphaned issues were reconciled manually via
`move_issue_to_done(filename, ticket_ids=[...])`, which moved them to
`.clasi/issues/done/` and set `status: done`.

## What Should Have Happened

When a sprint is created to deliver one or more pool issues, those issues should
be **linked to the sprint** at planning time, and at close the delivering
sprint should mark every linked issue `done` and move it into `done/` — so the
pool only ever contains genuinely-open work. Closing Sprint 001 should have left
exactly the two real follow-up issues pending, not four.

## Root Cause

**Emergent gap** (with a missing-instruction component). The happy path exists
but nothing enforces or reconciles it:

1. During Sprint 001 planning, the source issues were never linked to the
   sprint. No `link_sprint_issues(sprint_id, [...])` was called, and the issues
   were never moved into `<sprint>/issues/`.
2. `close_sprint`'s issue reconciliation only operates on issues that live under
   `<sprint>/issues/`. That directory was empty, so close had nothing to mark
   done and **succeeded silently**, leaving the originating issues orphaned.
3. Nothing in the team-lead "Execute Issues Through a Sprint" flow
   (`create_sprint` → sprint-planner → execute → close) requires the link step,
   and no close gate warns when pending pool issues were actually delivered by
   the sprint (e.g. via ticket `issue:` frontmatter refs).
4. As team-lead I compounded it: when I closed Sprint 001 I did not verify the
   source issues were marked done as part of the close.

A secondary, contributing confusion: the `move_issue_to_done` tool docstring
says "(no file move)", but for pool issues it **does** move the file into
`.clasi/issues/done/`. The misleading wording discourages using it for cleanup.

## Proposed Fix

Process changes (team-lead workflow + close gate):

1. **Mandatory link at sprint creation.** Add to the team-lead "Execute Issues
   Through a Sprint" workflow: immediately after `create_sprint`, call
   `link_sprint_issues(sprint_id, [issue_filenames])` for every issue the sprint
   will deliver. Treat an unlinked delivering-issue as a planning defect.
2. **Pre-close reconciliation gate.** Before `close_sprint`, verify every issue
   linked to the sprint is `status: done` and in `done/`. Close should refuse or
   loudly warn if a linked issue is still pending. (Mirror the existing
   `review_sprint_pre_close` checks.)
3. **Orphan guard at close.** Scan ticket `issue:` frontmatter refs and the
   pending pool; flag any issue the sprint's tickets reference that is still
   `pending`, so a sprint can't close while silently orphaning its own source
   issues.
4. **Fix the `move_issue_to_done` docstring** — it moves pool issues into
   `.clasi/issues/done/`; the "(no file move)" note is wrong for that case.

Items 2–4 require CLASI MCP/tooling changes; captured as a TODO. Item 1 is a
behavior change I will apply starting with the next sprint (see below — I will
link the two remaining issues to the new sprint at creation).

## Immediate Application

For the next sprint (delivering `hub-managed-led-reassert` and the motor
velocity ratio-drive issue), I will call `link_sprint_issues` right after
`create_sprint`, and verify both issues are moved to `done/` as part of close.
