# Branch Information

## Branch Name
```
feature/length-accuracy-frustum-plane-fixes
```

## Commit Message
```
feat: improve Length accuracy and fix Frustum/Plane bugs

- Replace overflow-scaling logic in Vector2D/3D::Length() with adaptive shift thresholds
- Add SortInPlace() method and use alpha-beta-gamma approximation for Turbo precision
- Use compile-time integral_constant to keep shifts efficient
- Fix Plane::FromPoints() divide-by-zero by checking cross product length before Normalize
- Fix Frustum::Update() camera position recovery (was using viewMatrix.Row3 incorrectly)
- Update documentation to reflect new precision modes (Accurate/Fast/Turbo)

Files changed:
- impl/fxp.hpp: Enhanced Accurate sqrt with hi/lo carry/borrow for full-range correctness
- impl/vector2d.hpp: Adaptive shift Length(), SortInPlace(), Turbo alpha-beta
- impl/vector3d.hpp: Adaptive shift Length(), SortInPlace(), Turbo alpha-beta-gamma
- impl/plane.hpp: Fix FromPoints() collinear point handling
- impl/frustum.hpp: Fix Update() world-space position calculation
```

## Pull Request Title
```
Improve Length calculation accuracy and fix geometric bugs
```

## Pull Request Description
```
Summary
- Replace overflow-scaling in Vector2D/3D::Length() with adaptive shift thresholds for better accuracy
- Add SortInPlace() and use alpha-beta-gamma approximations for Turbo precision
- Fix Plane::FromPoints() divide-by-zero on collinear points
- Fix Frustum::Update() incorrect camera position recovery

Details
- Vector2D/3D::Length(): Use adaptive shift based on absolute raw values OR to prevent overflow in Dot() intermediate
- Turbo precision: Fast alpha-beta-gamma approximation with sorted absolute components
- Plane::FromPoints(): Check cross product LengthSquared before Normalize to avoid divide-by-zero
- Frustum::Update(): Recover world-space camera position from view matrix rows instead of using Row3 directly

Files changed
- impl/fxp.hpp: Enhanced Accurate sqrt algorithm with hi/lo arithmetic
- impl/vector2d.hpp: New Length() implementation with SortInPlace()
- impl/vector3d.hpp: New Length() implementation with SortInPlace()
- impl/plane.hpp: Fixed collinear point handling
- impl/frustum.hpp: Fixed camera position calculation

Risks
- Low. Algorithmic improvements maintain compatibility while fixing edge cases
- Plane and Frustum fixes address specific bugs without API changes

Checklist
- [x] Length calculations handle overflow correctly across full range
- [x] Turbo precision provides fast approximation with acceptable error
- [x] Plane::FromPoints() handles collinear points safely
- [x] Frustum::Update() correctly recovers world-space position
```

## Git Commands
```bash
# Create and checkout new branch
git checkout -b feature/length-accuracy-frustum-plane-fixes

# Stage all changes
git add .

# Commit with message
git commit -m "feat: improve Length accuracy and fix Frustum/Plane bugs

- Replace overflow-scaling logic in Vector2D/3D::Length() with adaptive shift thresholds
- Add SortInPlace() method and use alpha-beta-gamma approximation for Turbo precision
- Use compile-time integral_constant to keep shifts efficient
- Fix Plane::FromPoints() divide-by-zero by checking cross product length before Normalize
- Fix Frustum::Update() camera position recovery (was using viewMatrix.Row3 incorrectly)
- Update documentation to reflect new precision modes (Accurate/Fast/Turbo)

Files changed:
- impl/fxp.hpp: Enhanced Accurate sqrt with hi/lo carry/borrow for full-range correctness
- impl/vector2d.hpp: Adaptive shift Length(), SortInPlace(), Turbo alpha-beta
- impl/vector3d.hpp: Adaptive shift Length(), SortInPlace(), Turbo alpha-beta-gamma
- impl/plane.hpp: Fix FromPoints() collinear point handling
- impl/frustum.hpp: Fix Update() world-space position calculation"

# Push to remote
git push -u origin feature/length-accuracy-frustum-plane-fixes
```
