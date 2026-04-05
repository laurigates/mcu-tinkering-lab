# Blueprint Development

This project uses Blueprint Development methodology for structured documentation and planning.

## Structure

| Directory | Purpose |
|-----------|---------|
| `manifest.json` | Version tracking and configuration |
| `feature-tracker.json` | Implementation progress tracking |
| `work-orders/` | Task packages for subagents |
| `ai_docs/` | Curated documentation for AI context |

## Related Directories

| Directory | Purpose |
|-----------|---------|
| `docs/prds/` | Product Requirements Documents |
| `docs/adrs/` | Architecture Decision Records |
| `docs/prps/` | Product Requirement Prompts |

## Commands

```
/blueprint:status          - Check version and configuration
/blueprint:derive-plans    - Derive docs from git history
/blueprint:derive-rules    - Derive rules from commit decisions
/blueprint:generate-rules  - Generate rules from PRDs
/blueprint:sync            - Check for stale generated content
```
