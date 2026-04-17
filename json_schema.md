# JSON Design Schema v1

## Top-Level Structure

```
{
  "schema_version": 1,
  "project_id": "Batch_2026_03_03",
  "bars": [ ... ]
}
```

### Fields

- `schema_version` — integer (must be `1` for now)
- `project_id` — string (optional but recommended)
- `bars` — array of bar objects

------

## Bar Object

```
{
  "bar_id": "B1",
  "length_mm": 2400,
  "joints": [ ... ]
}
```

### Fields

- `bar_id` — string (unique within file)
- `length_mm` — number (bar length in mm, > 0)
- `joints` — array of joint objects

------

## Joint Object

```
{
  "joint_id": "J1",
  "type": "T20",
  "subtype": "Female",
  "ori": "P",
  "position_mm": 120.0,
  "rotation_deg": 45.0
}
```

### Fields

- `joint_id` — string (unique within bar; used for logging/debugging)
- `type` — string (joint family, e.g. `"T20"`)
- `subtype` — string (variant within type, e.g. `"Male"`, `"Female"`)
- `ori` — string (`"P"` for positive or `"N"` for negative)
- `position_mm` — number (design-truth position along bar)
- `rotation_deg` — number (design-truth rotation at bar output)

### Derived key

The software concatenates `type` and `subtype` with an underscore to form
a lookup key used for offset and image matching in `settings.json`:

    type_key = f"{type}_{subtype}"     # e.g. "T20_Female"

------

## Execution Rules

1. **Sorting**
   - Software sorts `joints` by `position_mm` ascending before execution.
   - Small position = left on jig; large position = right on jig.
2. **No reliance on order in file**
   - Grasshopper export does not need to pre-sort.
3. **No index field** for joints
   - `joint_id` used in logs for traceability.
4. **Position domain**
   - Recommended constraint:
      `0 <= position_mm <= length`
   - Enforcement can be soft warning, not fatal.