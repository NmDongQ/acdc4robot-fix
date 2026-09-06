# cad2urdf — Fusion 360 CAD에서 sim2real용 URDF까지

**로봇** `body_GS_RL` (WRECKS) — 이족 digitigrade, 링크 12 / revolute 10 + fixed 1
**목표** Isaac Lab / MuJoCo에서 RL 학습 → 실물 이식(sim2real)
**최종 파이프라인** Fusion 360 → 패치된 ACDC4Robot → `fix_acdc_export.py` → `*_fixed.urdf`
**최종 갱신** 2026-09-06

이 문서는 두 번의 시도를 기록한다. 1차(2026-08, fusion2URDF)와 2차(2026-08~09, ACDC4Robot).
두 익스포터 모두 "성공"이라고 말하면서 틀린 URDF를 내놓았고, 결함의 종류가 놀랄 만큼
비슷했다. **CAD→URDF 익스포터의 출력은 반드시 수치로 검증해야 한다**는 것이 결론이다.

---

## TL;DR — 지금 URDF 뽑는 법

```bash
# 1. Fusion 360: ACDC4Robot(패치본) 실행 → URDF + Gazebo 선택 → export
# 2. 후처리 (약 4분, CoACD 포함)
cd ~/Desktop/projects/bipedal_droid/sim2real
python3 fix_acdc_export.py body_GS_RL_vN/body_GS_RL_vN.urdf \
        --rotate-links=base_link,robot_torso --roll=90
# 3. 결과: body_GS_RL_vN_fixed.urdf + body_GS_RL_vN_fixed.png (visual/collision/overlay)
```

`_fixed.urdf`를 meshes/ 폴더와 함께 Isaac/MuJoCo/뷰어에 넣으면 된다.
스크립트가 검증(트리·관성·좌우대칭)에 실패하면 파일을 쓰지 않고 `PROBLEM:`을 출력한다.

`--rotate-links ... --roll=90`은 이 로봇 전용이다 (base_link 컴포넌트 좌표계가 로봇
기준으로 90° 돌아가 있음, §2.6). 다른 로봇은 렌더 PNG를 보고 필요할 때만 준다.

아직 남은 것: 조인트 `effort`/`velocity`가 placeholder(10000)다. 실제 모터 스펙
(RS02, GIM8115)으로 바꿔야 한다.

---

## 1. 1차 시도 — fusion2URDF (2026-08, `~/Downloads/Rex/`)

익스포터 [Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF) v3.0.0.
상세는 `~/Downloads/Rex/FUSION2URDF_BUG.md`. 여기서는 요지만.

### 증상
export는 `PASS`, 에러 0. 그런데 **관절을 돌리면 자식 링크가 공간으로 날아갔다.**
정지 자세에서는 로봇처럼 보여서 눈치채기 어렵다. 렌더링에 안 보이는 관성 오류가 두 개 더 있었다.

### 결함 5가지와 수정

| # | 결함 | 원인 | 수정 |
|---|---|---|---|
| 1 | 한국어 Windows에서 export 크래시 | 템플릿 파일을 `encoding` 없이 읽음 (cp949) | `open(..., encoding='utf-8')` |
| 2 | 조인트 원점이 좌우 ±60 mm 어긋남 | 서브어셈블리 오프셋을 이미 월드 좌표인 값에 이중 적용 | 해당 분기 비활성화 |
| 3 | 메시·관성이 잘못된 프레임 | 메시는 occurrence `transform2`로 회전된 채 저장되는데 `<origin rpy>`는 항상 0 | `fix_mesh_origins.py`로 origin 재계산 |
| 4 | 좌우 관성이 100배 차이 | `com_global = position + com_local` — **occurrence 회전 무시**. 부품 원점에서 멀리 모델링된 구매부품(모터 커버)이 2 m 밖에 놓임 | `transform2`로 CoM·텐서 재배치 후 평행축 |
| 5 | 오프라인 재생성 시 `.obj` 참조 깨짐 | DAE 변환이 Fusion 안에서만 일어남 | 실재 파일로 참조 재지정 |

### 여기서 배운 것 (2차에도 그대로 적용됨)
- **질량과 CoM이 맞아도 관성 텐서는 틀릴 수 있다.** 결함 4는 질량 0.0 g 차이, 물리 타당성 검사 통과였는데 좌우 주축 관성이 100배 달랐다.
- **좌우 대칭 검사는 계산식과 독립인 물리적 필요조건**이라 순환 논증이 없다. "CAD 재계산 값과 대조"는 같은 공식을 쓰면 아무것도 증명 못 한다.
- 클래식 fusion2urdf 계열(syuntoku14 등 4종)은 `asBuiltJoints`를 안 읽어서 이 설계엔 쓸 수 없음을 확인했다.

### 왜 갈아탔나
fusion2URDF는 결함이 5개나 되고 로컬 패치가 많이 쌓였다. ACDC4Robot이 URDF/SDF/MJCF를
모두 지원하고 Autodesk App Store 공식 등록이라 2차에서 교체했다. (결과적으로 ACDC도 패치가 필요했다.)

---

## 2. 2차 시도 — ACDC4Robot (2026-08~09, `sim2real/`)

익스포터 [bionicdl-sustech/ACDC4Robot](https://github.com/bionicdl-sustech/ACDC4Robot).
설치 위치 `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/ACDC4Robot/`.

### 2.1 뷰어에 아무것도 안 보임 — 중첩 컴포넌트 처리 버그 (익스포터 패치)

**증상** URDF + meshes를 뷰어에 올려도 빈 화면. 처음엔 한글/중문 파일명(`구성요소54.stl`, `驱动盖.stl`)을 의심했으나 부차적이었다.

**진단**
```
links: 147 | joints: 11
joint-referenced links missing a <link> element: base_link, robot_left_leg_left_thigh, ... (12개 전부)
root links: 수십 개
```
- 조인트가 참조하는 **본체 링크 12개의 `<link>` 요소가 아예 없음**
- 대신 부품 135개가 트리에 연결 안 된 고아 링크로 export됨

**원인** `commands/ACDC4Robot/acdc4robot.py` `get_link_joint_list()`가 `root.allOccurrences`를
돌면서 "자식 컴포넌트가 없는 말단 occurrence"만 링크로 취급한다. 이 설계는 링크 컴포넌트
(thigh 등) 안에 모터·베어링이 하위 컴포넌트로 들어 있어서, 링크 자체는 건너뛰고 부품마다
링크가 됐다. 코드 주석에도 "nested components problem... not fully tested"라고 적혀 있다.

**실패한 우회** Fusion API 스크립트로 하위 컴포넌트를 바디로 평탄화(`FlattenLinks.py`) →
`moveToComponent`가 `InternalValidationError: cut_raw()`로 실패, `copyToComponent`+삭제로
바꾸니 **하위 컴포넌트 지오메트리에 앵커된 조인트 11개가 전부 사라짐.** 쓰지 말 것.

**수정 (익스포터 패치)** "조인트에 참여하는 occurrence = 링크"로 바꿨다. occurrence의
STL export와 `getPhysicalProperties`는 서브트리 전체를 포함하므로 중첩 구조를 그대로 두고
링크당 STL 1개, 질량 1개가 나온다.

```python
all_joints = []
seen_joint_names = set()
for joint in list(root.allJoints) + list(root.allAsBuiltJoints):
    if joint.name in seen_joint_names:   # 2.2 참조
        continue
    seen_joint_names.add(joint.name)
    all_joints.append(joint)

if all_joints:
    seen = set()
    for joint in all_joints:
        joint_list.append(Joint(joint))
        for occ in (joint.occurrenceOne, joint.occurrenceTwo):
            if occ is None or occ.fullPathName in seen:
                continue
            seen.add(occ.fullPathName)
            link_list.append(Link(occ))
else:
    # 조인트가 없는 디자인은 기존 방식(말단 occurrence)으로
    ...
```

**검증** 링크 12 / 조인트 11 / 루트 base_link 1개 / 좌우 질량 대칭 / 총질량이 부품별 합
14.58 kg과 일치 (→ 서브트리 물성 포함 확인) / MuJoCo 로드 + 렌더 정상.

### 2.2 조인트 중복 (v2에서만 발생)

v2 export에 11개 조인트가 **동일 값으로 2번씩** 들어가 22개. 링크에 부모 조인트가 둘이면
트리가 깨진다. 후처리에서 이름 기준 중복 제거 + 익스포터에도 같은 방어를 넣었다.
v3에서는 재발하지 않았다 (Fusion 쪽 원인은 미확인 — 디자인 복사 과정 의심).

### 2.3 MuJoCo 로드 실패 — 메시 20만 페이스 초과

```
stl_decoder: number of faces should be between 1 and 200000 in STL file
'meshes/robot_left_leg_left_hip_roll.stl'
```
왼쪽 힙롤에만 고정밀 모터 CAD(254,556 페이스)가 들어 있었다. 후처리에서 quadric
decimation으로 12만 페이스로 줄인다 (`trimesh` + `fast-simplification`).

### 2.4 뷰어에서 base_link가 90° 돌아 보임 — Fusion 컴포넌트 좌표계

처음엔 "뷰어 버그(루트 링크 visual origin 미적용)"라고 가정하고 더미 루트 링크를 얹어봤으나
**바뀌지 않았다.** MuJoCo와 뷰어가 같은 그림을 그리므로 데이터가 그런 것이다.

**원인** Fusion에서 base_link 컴포넌트 자체 좌표축이 로봇 기준으로 roll 90° 돌아가 있다.
ACDC는 컴포넌트 좌표계를 그대로 믿는다. torso(fixed, 원점 동일)도 같이 돌아가 있다.

**수정** 링크 프레임(조인트가 걸리는 기준)은 그대로 두고, 그 링크의 visual/collision/inertial
**origin에만** `R_fix`를 왼쪽에서 곱한다. 조인트는 안 건드리므로 다리는 그대로.
```
R_new = R_fix · R_old,  xyz_new = R_fix · xyz_old
```
`+90°`와 `−90°` 후보를 둘 다 렌더링해서 "실린더 두 개가 앞(발끝 방향)을 봐야 한다"는
설계 의도로 +90°를 골랐다. 근본 수정은 Fusion에서 컴포넌트 origin을 재정렬하는 것.

### 2.5 base_link 질량이 4 kg? — fixed 조인트 병합 (버그 아님)

CAD 1.7 kg인데 뷰어가 4.03 kg을 보여줬다. URDF에는 base 1.766 + torso 2.261이 따로
들어 있고, 뷰어/MuJoCo/Isaac이 **fixed 조인트로 붙은 링크를 하나의 강체로 병합**해서
표시한 것. 물리적으로 맞는 동작이다. (MuJoCo `fusestatic`, Isaac "Merge Fixed Joints")

같은 맥락: MuJoCo가 베이스를 월드에 고정하면 base+torso 질량이 총합에서 빠져 보인다.
freejoint를 붙이면 돌아온다.

### 2.6 inertia 박스가 base/torso만 어긋남 — ACDC 루트 링크 버그 (핵심)

**증상** 뷰어의 inertia 시각화에서 다리는 메시에 맞는데 base_link/torso 박스만 이상한 방향.

**1차 원인 (내 수정의 부작용)** §2.4 회전을 inertial origin의 `rpy`에 넣었는데,
robotsfan 뷰어는 inertial origin의 회전을 무시하고 텐서를 링크 축 기준으로 그린다.
→ 회전을 **텐서에 접어 넣고**(`I' = R·I·Rᵀ`) rpy는 0으로 유지하도록 바꿈. 어떤
뷰어/시뮬레이터에서도 동일하게 해석된다.

**2차 원인 (ACDC 버그)** 그래도 base_link만 남았다. 메시로부터 균일밀도 CoM을 계산해
URDF와 비교하니 **base_link만 y부호가 반대** (메시 +0.029 vs URDF −0.041). 다리·torso는
부호 일치.

ACDC 코드를 보면 루트 링크에 대해:
- `urdf.py:698` visual origin = `link.pose` → **링크 프레임 = Fusion 월드**, 메시를 컴포넌트 포즈만큼 이동
- `link.py get_CoM_sdf()` inertial = `L_R_w · (CoM_w − Lo_w)` → **링크 프레임 = 컴포넌트 자체 좌표계**

두 기준이 달라서 루트 링크의 CoM/텐서만 컴포넌트 포즈의 회전(−90° roll)만큼 어긋난다.
자식 링크는 부모 조인트 프레임 기준으로 일관되게 계산돼 멀쩡하고, torso는 포즈가
단위행렬에 가까워 우연히 맞았다. **원본 export부터 있던 버그다.**

**수정 (후처리)** 루트 링크의 inertial을 visual과 같은 프레임으로 되돌린다:
```
R, t = 루트 링크 visual origin
CoM' = R · CoM + t
I'   = R · I · Rᵀ
```
수정 후 CoM 부호가 메시와 일치 (0, −0.020, 0.029) vs (0, −0.041, 0.019). 크기 차이는
모터(고밀도)가 한쪽에 몰린 정상적 차이.

**참고** base_link는 관성모멘트 세 값이 거의 같아서(0.0113/0.0106/0.0102) 주축 방향이
수학적으로 불안정하다. 뷰어에서 박스가 ~20° 기울어 보여도 물리적으로 무의미하다.

### 2.7 torso inertia 박스 좌우 비대칭 — CAD가 실제로 비대칭 (버그 아님)

메시만으로 계산한 텐서에서도 같은 부호의 `ixy`, `ixz`가 나온다. Fusion의 torso 컴포넌트
안에 중심에서 벗어난 부품(커넥터/브래킷/배터리 오프셋)이 있다는 뜻. 커플링이 ixx의 1.7%지만
ixx≈iyy라 박스가 6~7° 요로 틀어져 보인다. 시뮬레이션 영향 무시 가능. CAD에서 확인 권장.

### 2.8 총질량 14.58 → 9.89 kg (v2 → v3)

v3에서 hip_roll(1.49→0.47)과 thigh(2.36→1.45)에서 각각 1 kg 가까이 빠졌다. 모터 하나
무게라, **모터 부품이 링크 컴포넌트 밖으로 나갔거나 숨김/삭제**됐을 가능성. 패치된
익스포터는 "조인트에 참여하는 컴포넌트 안"만 내보내므로 바깥 부품은 **경고 없이 빠진다.**
→ Fusion 전체 어셈블리 질량과 대조할 것. (미확인)

---

## 3. Collision 형상

익스포터는 visual 메시를 collision에 그대로 복사한다 (8만~12만 페이스). 시뮬레이터는
메시 collision을 convex hull로 감싸므로 오목한 부분이 메워져 실제보다 뚱뚱해지고,
자가충돌(self-collision)도 이 형상으로 판정하기 때문에 영점 자세부터 인접 링크가 겹친다.

세 단계로 발전시켰다. 전부 `fix_acdc_export.py --collision=` 옵션으로 남아 있다.

| 방식 | 내용 | 결과 |
|---|---|---|
| box | 링크당 OBB/AABB 1개 (부피 이득 30% 미만이면 축 정렬) | 빠르지만 투박, 자가충돌엔 부적합 |
| `primitive` | Rex fusion2URDF의 `fit_primitive` 규칙 이식: 치수 패턴 → 부피 충전율 → revolute 조인트 축 힌트. **단면이 원형일 때만** cylinder | calf만 cylinder, 나머지 box. Rex에선 foot도 cylinder였는데(조인트 힌트 무검증) 발바닥이 구르므로 보행엔 나쁨 |
| `coacd` (기본) | [CoACD](https://github.com/SarahWeiii/CoACD) convex decomposition, 링크당 ≤12 조각, 조각당 ≤64 정점 (`decimate=True` 필요 — 없으면 `max_ch_vertex`가 무시됨) | 메시 형상을 거의 그대로 따라감. 자가충돌 판정에 적합 |

**연산 부담** 조각당 64정점 제한이 핵심이다. PhysX(Isaac)는 convex mesh를 64정점 이하로
cooking하므로 그 이상은 어차피 깎인다. 총 142 조각, 약 2만 페이스. MuJoCo는 부모-자식
링크 충돌을 기본 제외하고, Isaac Lab 로봇들(G1 등)도 같은 방식이라 수천 환경 병렬에도 문제없다.
느리면 `--coacd-max-hulls 8` 또는 몸통만 `primitive`로.

**중간에 잡은 버그** 회전행렬→rpy 변환이 pitch=±90°(짐벌락)에서 틀려서 calf 실린더가
옆으로 누웠다. `scipy.spatial.transform.Rotation.as_euler('xyz')`로 교체.

---

## 4. 검증 — 눈으로는 못 잡는다

`fix_acdc_export.py`가 매번 자동으로 돌리는 검사:

| 검사 | 잡는 것 |
|---|---|
| 조인트 참조 링크 존재, 루트 1개, 다중 부모 없음 | §2.1, §2.2 |
| 관성 양의정부호 + I₁+I₂ ≥ I₃ | 계산 오류 |
| **좌우 주축 관성 대칭 < 3%** | Rex 결함 4 같은 "질량은 같은데 관성 100배" — 이번엔 최대 1.37% |
| MuJoCo 로드 + 3뷰 렌더 (visual / collision / overlay) | 형상·프레임·collision 어긋남 |

수동으로 한 것:
- **메시 기반 CoM/텐서와 URDF 비교** (균일밀도 가정, 부호와 주축 정렬만 봄) — §2.6을 잡았다
- 두 개의 독립 파서(MuJoCo + 웹 뷰어)로 같은 그림이 나오면 뷰어 버그가 아니다 — §2.4
- Fusion 물성값과 총질량 대조 — §2.8

---

## 5. 파일

```
sim2real/
├── cad2urdf.md                 ← 이 문서
├── fix_acdc_export.py          ← 후처리 스크립트 (일반화됨, --help)
├── body_GS_RL/                 ← 첫 export (깨진 것, 참고용)
├── body_GS_RL_v2/              ← 조인트 중복 있던 버전
└── body_GS_RL_v3/
    ├── body_GS_RL_v3.urdf          원본 export
    ├── body_GS_RL_v3_fixed.urdf    ★ 사용할 파일
    ├── body_GS_RL_v3_fixed.png     검증 렌더
    └── meshes/ (+ collision/)      visual STL + CoACD 조각
```

익스포터 패치: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/ACDC4Robot/commands/ACDC4Robot/acdc4robot.py`
— App Store에서 업데이트하면 사라진다. 공개 저장소(§6)에 패치 파일로 보관.

이전 시도 자료: `~/Downloads/Rex/FUSION2URDF_BUG.md`, `urdf_render/check_inertia.py`

---

## 6. 공개

패치와 스크립트, 이 문서의 영문판을 GitHub에 공개하고 ACDC4Robot 업스트림에 이슈/PR로
보고했다. 저장소: https://github.com/NmDongQ/acdc4robot-fix
