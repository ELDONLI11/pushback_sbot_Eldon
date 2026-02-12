# 🤖 SBOT Competition Quick Reference Card

## V5 Brain Program Slots

| Slot | Name | What It Does | When to Use |
|------|------|--------------|-------------|
| **1** | sbot-selector | Selector working (use D-pad + A) | **TRY THIS FIRST** - Normal matches |
| **2** | leftmid | Red/Blue LEFT (Middle Goal) | Selector failed, Left start |
| **3** | rightlow | Red/Blue RIGHT (Low Goal) | Selector failed, Right start |
| **4** | leftsolo | Solo AWP (Red Left) | Solo AWP matches |
| **5** | skills | Skills Autonomous | Skills runs |

---

## 📋 Match Procedure

### ✅ Preferred: Use Slot 1 (Selector)

1. Select **Slot 1** on brain
2. Power on → wait for controller display
3. **D-pad LEFT** = Red/Blue Left (default)
4. **D-pad RIGHT** = Red/Blue Right
5. Press **A** to confirm
6. Match starts → runs your selected mode

### 🆘 Backup: Selector Failed

1. **Ask ref for time** to switch program
2. Select appropriate backup slot (2, 3, 4, or 5)
3. Power on → **no button input needed**
4. Match starts → runs automatically

---

## 🎯 Which Slot for Each Start Position?

```
RED ALLIANCE:
┌─────────────────────────────────────┐
│                                     │
│  RED LEFT          RED RIGHT        │
│  Use Slot 2        Use Slot 3       │
│  (leftmid)         (rightlow)       │
│                                     │
└─────────────────────────────────────┘

BLUE ALLIANCE:
┌─────────────────────────────────────┐
│                                     │
│  BLUE RIGHT        BLUE LEFT        │
│  Use Slot 2        Use Slot 3       │
│  (leftmid)         (rightlow)       │
│                                     │
└─────────────────────────────────────┘
```

**Remember**: Left/Right are SAME for Red and Blue!
- **Left side** (Slot 2) scores Middle Goal first
- **Right side** (Slot 3) scores Low Goal first

---

## 🔧 Before Competition Checklist

- [ ] All 5 slots uploaded to brain (run `.\upload-backup-slots.ps1`)
- [ ] Tested Slot 1 selector on practice field
- [ ] Tested Slot 2 (leftmid) autonomous
- [ ] Tested Slot 3 (rightlow) autonomous
- [ ] Know how to switch slots quickly
- [ ] Driver knows selector controls
- [ ] Battery fully charged
- [ ] Controller paired and charged

---

## 📞 Emergency Contacts

Programmer: ___________________

Phone: ___________________

---

**Print this page and keep it at the competition table!**
