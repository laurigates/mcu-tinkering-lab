# GitHub Issues for Robocar Simulation Improvements

This directory contains prepared issue descriptions for improvements to the robocar simulation project.

## Quick Create on GitHub

You can create these issues by copying the content from each `.md` file and pasting it into GitHub's "New Issue" form at:
https://github.com/laurigates/mcu-tinkering-lab/issues/new

## Issues Overview

### 1. Fix Physics Calibration (Priority: High ⭐⭐⭐)
**File**: `issue-1-physics-calibration.md`
**Effort**: 1-2 hours
**Impact**: Validates entire simulation, enables 24/24 tests passing

### 2. Enhance 2D Visualization (Priority: Medium ⭐⭐)
**File**: `issue-2-enhance-visualization.md`
**Effort**: 2-3 hours
**Impact**: Better debugging and demo experience

### 3. Integration Testing (Priority: High ⭐⭐⭐)
**File**: `issue-3-integration-testing.md`
**Effort**: 2-3 hours
**Impact**: Confidence in hardware integration

### 4. Example Scripts & Docs (Priority: Medium ⭐⭐)
**File**: `issue-4-example-scripts.md`
**Effort**: 1-2 hours
**Impact**: Better onboarding and adoption

### 5. Camera & Vision Features (Priority: Medium ⭐⭐)
**File**: `issue-5-camera-vision.md`
**Effort**: 3-4 hours
**Impact**: Complete simulation environment for CV testing

## Recommended Order

Based on impact and dependencies:
1. **Issue 1** - Physics Calibration (foundational)
2. **Issue 3** - Integration Testing (validates everything)
3. **Issue 2** - Visualization (helps with debugging)
4. **Issue 4** - Examples & Docs (helps adoption)
5. **Issue 5** - Camera Vision (advanced feature)

## Alternative: Batch Create with GitHub CLI

If you have `gh` CLI installed and authenticated:

```bash
cd /home/user/mcu-tinkering-lab/packages/esp32-projects/robocar-simulation/.github-issues

# Create all issues
for file in issue-*.md; do
    title=$(head -n 1 "$file" | sed 's/^# //')
    body=$(tail -n +3 "$file")
    labels=$(grep "^**Labels**:" "$file" | sed 's/^**Labels**: //; s/`//g')

    gh issue create --title "$title" --body "$body" --label "$labels"
done
```

## Manual Creation Steps

1. Go to https://github.com/laurigates/mcu-tinkering-lab/issues/new
2. Copy the title (first line without `#`) from the `.md` file
3. Copy the entire content below the title line into the description
4. Add labels mentioned in the **Labels** line
5. Click "Submit new issue"
6. Repeat for each file

## Notes

- Each issue has checkboxes for task tracking
- Issues include impact assessment and effort estimates
- All issues reference the original PR #43 where appropriate
- Issues are standalone but can be linked if worked on together
