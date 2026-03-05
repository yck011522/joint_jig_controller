# JSON Design Schema v1 (Revised)

## Top-Level Structure

```
{
  "schema_version": 1,
  "project_id": "Batch_2026_03_03",
  "tubes": [ ... ]
}
```

### Fields

- `schema_version` — integer (must be `1` for now)
- `project_id` — string (optional but recommended)
- `tubes` — array of tube objects

------

## Tube Object

```
{
  "id": "TUBE_001",
  "length": 2400,
  "joints": [ ... ]
}
```

### Fields

- `id` — string (unique within file)
- `length` — number (> 0)
- `joints` — array of joint objects

------

## Joint Object (Final v1 Definition)

```
{
  "id": "J001",
  "type": "A",
  "ori": "L",
  "position_mm": 120.0,
  "rotation_deg": 45.0
}
```

### Fields

- `id` — string (unique within tube; used for logging/debugging)
- `type` — string (single character)
- `ori` — string (single character; L/R likely, but not enforced yet)
- `position_mm` — number (design-truth position along tube)
- `rotation_deg` — number (design-truth rotation at tube output)

------

## Execution Rules (Important)

1. **Sorting**
   - Software will sort `joints` by `position_mm` ascending before execution.
   - Small position = left on jig; large position = right on jig.
2. **No reliance on order in file**
   - Grasshopper export does not need to pre-sort.
3. **No index field** for joints
   - `id` used in logs for traceability.
4. **Position domain**
   - Recommended constraint:
      `0 <= position_mm <= length`
   - Enforcement can be soft warning, not fatal.