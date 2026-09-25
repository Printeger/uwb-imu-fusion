P0-06 第六轮独立验收：**FAILED（可修复，非 BLOCKED）**。

阻断项：
1. 130/index128 O07 用例缩减了 hypothesis/action coverage，无法证明可信的 pre-risk winner。
- test:1316-1319使用129条全部plausible evidence生成130 actions。
- :1349-1358将完整集合替换为手工{target,residual}并把residual标为非plausible。
- :1378-1406的PL只消费projected_remaining{residual}。
- :1419-1421的FDE也只消费这两条hypothesis/evidence。
- 因此:1442-1443 action129 UseInReferenceEstimate是删除其余127个原plausible义务后得到。保持同一129条plausible时，该单mode action不覆盖完整集合。
违反production router/PL/FDE同一完整非空集合及不得减少coverage。
2. 130用例缺独立uncapped recoverability oracle。pass_count==1仅由同一production router/post/PL链自检；raw-SVD只在54-cell。不能独立证明action129唯一pre-risk recoverable，summary complete_plausible_census/recoverable声明不足。

权威总表O07允许same winner/refusal，最终RiskBudgetInvalid无selection/commit合法诚实；失败不是没强造recovery，绝不能risk override/伪qualification/改预算阈值。

最小修复：从一开始用同一个完整target/residual集合构造130 raw actions，不先129 plausible再缩减；router/post/非空PL/FDE始终消费一致完整集合；独立Dense/raw-SVD证明action129唯一pre-risk recoverable；三种reorder比完整refusal/state/cov/PL/coverage/risk/disposition，cap128仍SEARCH_INCOMPLETE。

其余normal clean/full/tests、directed24/24、54-cell、integrity_v2/FDE/P0-04/P0-05/round2/CTest29/29、ABI、五hash、13-path patch、risk override/env hook删除、exception staging/rank fallback主体均PASS。

附带证据缺口：真实pipeline异常测试是keep-only，无动态duplicate_of，CSV只抽查一个occurrence。建议补带exact duplicate的真实pipeline异常场景并逐行核final packet/integrity.csv/candidates.csv。
Reviewer未修改工作树/总表/commit/tag。
