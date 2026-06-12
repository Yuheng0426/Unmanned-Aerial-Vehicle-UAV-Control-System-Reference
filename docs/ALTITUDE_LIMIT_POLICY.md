# Altitude Limit Policy

This repository includes a locked reference altitude limit:

```text
SafetyConfig::locked_legal_altitude_limit_m = 120.0
```

This value is intended as a conservative software boundary for an educational simulator. It must not be raised, bypassed, disabled, or weakened in downstream adaptations.

## Legal Responsibility

Local law always takes priority. Some regions, sites, aircraft categories, operators, or missions may require a lower maximum altitude. If local rules require a lower limit, use the lower limit.

This repository does not grant permission to fly, does not replace aviation regulations, and does not certify any aircraft.

## Why the Limit Is Locked

The altitude boundary is not exposed as a normal runtime setting because casual changes can create unsafe or illegal behavior. The code clamps commanded altitude to this limit and triggers landing behavior if measured altitude exceeds it.

## If Someone Changes It Anyway

Changing this limit is strongly discouraged. Anyone who raises, bypasses, disables, or weakens the altitude limit accepts full responsibility for legal violations, property damage, injury, flyaway events, airspace conflicts, or other consequences.

## Safer Alternatives

- Keep the locked limit unchanged.
- Add a lower project-specific limit if your local rules require one.
- Test altitude behavior in simulation only.
- Never fly near restricted airspace, airports, people, roads, buildings, or unsafe environments.
