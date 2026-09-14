#pragma once

// Application-layer orchestration for the paper runner. The executable entry
// point stays deliberately thin; estimator and IE mathematics remain in the
// uifgo library modules it invokes.
int RunIePaperApplication(int argc, char** argv);
