#define XP 0
#define XN 1
#define YP 2
#define YN 3
#define ZP 4
#define ZN 5
//X Y Z
//Left Above Front
#define LAF ((1<<XN) | (1<<YP) | (1<<ZN))
//Left Above Middle
#define LAM (  (1<<XN) | (1<<YP))
//Left Above Back
#define LAB ((1<<XN) | (1<<YP) | (1<<ZP))
//Left Middle Front
#define LMF ((1<<XN) | (1<<ZN))
//Left Middle Middle
#define LMM ((1<<XN))
//Left Middle Back
#define LMB ((1<<XN) | (1<<ZP))
//Left Bottom Front
#define LBF ((1<<XN) | (1<<YN) | (1<<ZN))
//Left Bottom Middle
#define LBM ((1<<XN) | (1<<YN))
//Left Bottom Back
#define LBB ((1<<XN) | (1<<YN) | (1<<ZP))

//Middle Above Front
#define MAF ((1<<YP) | (1<<ZN))
//Middle Above Middle
#define MAM ((1<<YP))
//Middle Above Back
#define MAB ((1<<YP) | (1<<ZP))
//Middle Middle Front
#define MMF ((1<<ZN))
//Middle Middle Middle
#define MMM (0)
//Middle Middle Back
#define MMB ((1<<ZP))
//Middle Bottom Front
#define MBF ((1<<YN) | (1<<ZN))
//Middle Bottom Middle
#define MBM (1<<YN)
//Middle Bottom Back
#define MBB ((1<<YN) | (1<<ZP))

//Right Above Front
#define RAF ((1<<XP) | (1<<YP) | (1<<ZN))
//Right Above Middle
#define RAM ((1<<XP) | (1<<YP))
//Right Above Back
#define RAB ((1<<XP) | (1<<YP) | (1<<ZP))
//Right Middle Front
#define RMF ((1<<XP) | (1<<ZN))
//Right Middle Middle
#define RMM (1<<XP)
//Right Middle Back
#define RMB ((1<<XP) | (1<<ZP))
//Right Bottom Front
#define RBF ((1<<XP) | (1<<YN) | (1<<ZN))
//Right Bottom Middle
#define RBM ((1<<XP) | (1<<YN))
//Right Bottom Back
#define RBB ((1<<XP) | (1<<YN) | (1<<ZP))

