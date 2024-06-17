#include "DxLib.h"
#include <malloc.h>
#include <stdio.h>

#define MOVESPEED			(20.0)			// �ړ����x
#define SPHERESIZE			(200.0f)		// ���̂̃T�C�Y
#define COLLWIDTH			(400.0f)		// �����蔻��̃T�C�Y

// �|���S�����m�̘A������ۑ�����ׂ̍\����
struct POLYLINKINFO
{
	int LinkPolyIndex[3];			// �|���S���̎O�̕ӂƂ��ꂼ��אڂ��Ă���|���S���̃|���S���ԍ�( -1�F�אڃ|���S������  -1�ȊO�F�|���S���ԍ� )
	float LinkPolyDistance[3];			// �אڂ��Ă���|���S���Ƃ̋���
	VECTOR CenterPosition;				// �|���S���̒��S���W
};

// �o�H�T�������p�̂P�|���S���̏��
struct PATHPLANNING_UNIT
{
	int PolyIndex;					// �|���S���ԍ�
	float TotalDistance;				// �o�H�T���ł��̃|���S���ɓ��B����܂łɒʉ߂����|���S���Ԃ̋����̍��v
	float NormDistance;				// 0.0 ~ 1.0 �ɐ��K����������(�]���l)
	int PrevPolyIndex;				// �o�H�T���Ŋm�肵���o�H��̈�O�̃|���S��( ���|���S�����o�H��ɖ����ꍇ�� -1 )
	int NextPolyIndex;				// �o�H�T���Ŋm�肵���o�H��̈��̃|���S��( ���|���S�����o�H��ɖ����ꍇ�� -1 )
	PATHPLANNING_UNIT* ActiveNextUnit;		// �o�H�T�������ΏۂɂȂ��Ă��鎟�̃|���S���̃������A�h���X���i�[����ϐ�
};

// �o�H�T�������Ŏg�p�������ۑ�����ׂ̍\����
struct PATHPLANNING
{
	VECTOR StartPosition;				// �J�n�ʒu
	VECTOR GoalPosition;				// �ڕW�ʒu
	PATHPLANNING_UNIT* UnitArray;			// �o�H�T�������Ŏg�p����S�|���S���̏��z�񂪊i�[���ꂽ�������̈�̐擪�������A�h���X���i�[����ϐ�
	PATHPLANNING_UNIT* ActiveFirstUnit;		// �o�H�T�������ΏۂɂȂ��Ă���|���S���Q�̍ŏ��̃|���S�����ւ̃������A�h���X���i�[����ϐ�
	PATHPLANNING_UNIT* StartUnit;			// �o�H�̃X�^�[�g�n�_�ɂ���|���S�����ւ̃������A�h���X���i�[����ϐ�
	PATHPLANNING_UNIT* GoalUnit;			// �o�H�̃S�[���n�_�ɂ���|���S�����ւ̃������A�h���X���i�[����ϐ�
};

// �T�������o�H���ړ����鏈���Ɏg�p�������Z�߂��\����
struct PATHMOVEINFO
{
	int NowPolyIndex;				// ���ݏ���Ă���|���S���̔ԍ�
	VECTOR NowPosition;				// ���݈ʒu
	VECTOR MoveDirection;				// �ړ�����
	PATHPLANNING_UNIT* NowPathPlanningUnit;	// ���ݏ���Ă���|���S���̌o�H�T����񂪊i�[����Ă��郁�����A�h���X���i�[����ϐ�
	PATHPLANNING_UNIT* TargetPathPlanningUnit;	// ���̒��Ԓn�_�ƂȂ�o�H��̃|���S���̌o�H�T����񂪊i�[����Ă��郁�����A�h���X���i�[����ϐ�
};

const float INF_COST = (1 << 29);

// PolyList
int CheckOnPolyIndex(VECTOR Pos, MV1_REF_POLYGONLIST& PolyList);						// �w��̍��W�̒����A�Ⴕ���͒���ɂ���|���S���̔ԍ����擾����( �|���S�������������ꍇ�� -1 ��Ԃ� )

// PolyLinkInfo
void SetupPolyLinkInfo(int StageModelHandle, MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo);						// �|���S�����m�̘A�������\�z����
void CalculateCenterPosition(MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo);
void SettingPolyLinkInfo(MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo);
void FindLinkPolygon(int polygonIndex, MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo, POLYLINKINFO* PLInfo, MV1_REF_POLYGON* RefPoly);
void AddPolyLinkInfo(int polygonIndex, VECTOR CenterPosition, POLYLINKINFO* PLInfo, MV1_REF_POLYGON* RefPoly, MV1_REF_POLYGON* RefPolySub);
void TerminatePolyLinkInfo(POLYLINKINFO* PolyLinkInfo);						// �|���S�����m�̘A�����̌�n�����s��

bool CheckPolyMove(VECTOR StartPos, VECTOR TargetPos, POLYLINKINFO* PolyLinkInfo, MV1_REF_POLYGONLIST& PolyList);			// �|���S�����m�̘A�������g�p���Ďw��̓�̍��W�Ԃ𒼐��I�Ɉړ��ł��邩�ǂ������`�F�b�N����( �߂�l  true:�����I�Ɉړ��ł���  false:�����I�Ɉړ��ł��Ȃ� )
bool CheckPolyMoveWidth(VECTOR StartPos, VECTOR TargetPos, float Width, POLYLINKINFO* PolyLinkInfo, MV1_REF_POLYGONLIST& PolyList);	// �|���S�����m�̘A�������g�p���Ďw��̓�̍��W�Ԃ𒼐��I�Ɉړ��ł��邩�ǂ������`�F�b�N����( �߂�l  true:�����I�Ɉړ��ł���  false:�����I�Ɉړ��ł��Ȃ� )( ���w��� )

// PathPlanning
void MapSearch(VECTOR PositionList[4], PATHPLANNING PathPlanning[4], PATHPLANNING influenceMap[3], MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo);
void DrawEvaluationMap(PATHPLANNING& PathPlanning, MV1_REF_POLYGONLIST& PolyList);
bool SetupPathPlanning(VECTOR StartPos, VECTOR GoalPos, MV1_REF_POLYGONLIST& PolyList, PATHPLANNING& PathPlanning, POLYLINKINFO* PolyLinkInfo);			// �w��̂Q�_�̌o�H��T������( �߂�l  true:�o�H�\�z����  false:�o�H�\�z���s( �X�^�[�g�n�_�ƃS�[���n�_���q���o�H������������ ) )
void TerminatePathPlanning(PATHPLANNING PathPlanning[4]);						// �o�H�T�����̌�n��

// PathMove
void MoveInitialize(bool isEscale[3], VECTOR PositionList[4], MV1_REF_POLYGONLIST& PolyList, PATHMOVEINFO PathMove[3], PATHPLANNING influenceMap[3], POLYLINKINFO* PolyLinkInfo);							// �T�������o�H���ړ����鏈���̏��������s���֐�
void MoveProcess(bool isEscale[3], VECTOR PositionList[4], MV1_REF_POLYGONLIST& PolyList, PATHMOVEINFO PathMove[3], PATHPLANNING influenceMap[3], POLYLINKINFO* PolyLinkInfo);							// �T�������o�H���ړ����鏈���̂P�t���[�����̏������s���֐�
void RefreshMoveDirectionMap(int index, bool isEscape, PATHMOVEINFO& PathMove, PATHPLANNING influenceMap, POLYLINKINFO* PolyLinkInfo);					// �T�������o�H���ړ����鏈���ňړ��������X�V���鏈�����s���֐�( �߂�l  true:�S�[���ɒH�蒅���Ă���  false:�S�[���ɒH�蒅���Ă��Ȃ� )
bool RefreshMoveDirection(PATHMOVEINFO& PathMove, PATHPLANNING& PathPlanning, POLYLINKINFO* PolyLinkInfo, MV1_REF_POLYGONLIST& PolyList);						// �T�������o�H���ړ����鏈���ňړ��������X�V���鏈�����s���֐�( �߂�l  true:�S�[���ɒH�蒅���Ă���  false:�S�[���ɒH�蒅���Ă��Ȃ� )

void SetActionFlag(bool isEscape[3]);

// �w��̍��W�̒����A�Ⴕ���͒���ɂ���|���S���̔ԍ����擾����( �|���S�������������ꍇ�� -1 ��Ԃ� )
int CheckOnPolyIndex(VECTOR Pos, MV1_REF_POLYGONLIST& PolyList)
{
	int i;
	VECTOR LinePos1;
	VECTOR LinePos2;
	HITRESULT_LINE HitRes;
	MV1_REF_POLYGON* RefPoly;

	// �w��̍��W��Y�������ɑ傫���L�т�����̂Q���W���Z�b�g
	LinePos1 = VGet(Pos.x, 1000000.0f, Pos.z);
	LinePos2 = VGet(Pos.x, -1000000.0f, Pos.z);

	// �X�e�[�W���f���̃|���S���̐������J��Ԃ�
	RefPoly = PolyList.Polygons;
	for (i = 0; i < PolyList.PolygonNum; i++, RefPoly++)
	{
		// �����Ɛڂ���|���S�����������炻�̃|���S���̔ԍ���Ԃ�
		HitRes = HitCheck_Line_Triangle(
			LinePos1,
			LinePos2,
			PolyList.Vertexs[RefPoly->VIndex[0]].Position,
			PolyList.Vertexs[RefPoly->VIndex[1]].Position,
			PolyList.Vertexs[RefPoly->VIndex[2]].Position
		);
		if (HitRes.HitFlag)
		{
			return i;
		}
	}

	// �����ɗ���������Ɛڂ���|���S�������������Ƃ������ƂȂ̂� -1 ��Ԃ�
	return -1;
}


// �|���S�����m�̘A�������\�z����
void SetupPolyLinkInfo(int StageModelHandle, MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo)
{
	int i;
	int j;
	POLYLINKINFO* PLInfo;
	POLYLINKINFO* PLInfoSub;
	MV1_REF_POLYGON* RefPoly;
	MV1_REF_POLYGON* RefPolySub;

	// �S�|���S���̒��S���W���Z�o
	//CalculateCenterPosition(PolyList, PolyLinkInfo);

	// �S�|���S���̒��S���W���Z�o
	CalculateCenterPosition(PolyList, PolyLinkInfo);

	// �|���S�����m�̗אڏ��̍\�z
	SettingPolyLinkInfo(PolyList, PolyLinkInfo);

}

void CalculateCenterPosition(MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo)
{
	int i;
	int polygonNum = PolyList.PolygonNum;
	MV1_REF_POLYGON* stagePolygons = PolyList.Polygons;
	POLYLINKINFO* PLInfo = PolyLinkInfo;

	for (i = 0; i < polygonNum; i++, PLInfo++, stagePolygons++)
	{
		PLInfo->CenterPosition =
			VScale(VAdd(PolyList.Vertexs[stagePolygons->VIndex[0]].Position,
				VAdd(PolyList.Vertexs[stagePolygons->VIndex[1]].Position,
					PolyList.Vertexs[stagePolygons->VIndex[2]].Position)), 1.0f / 3.0f);
	}
}

void SettingPolyLinkInfo(MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo)
{
	int i;
	POLYLINKINFO* PLInfo;
	MV1_REF_POLYGON* RefPoly;

	PLInfo = PolyLinkInfo;
	RefPoly = PolyList.Polygons;
	for (i = 0; i < PolyList.PolygonNum; i++, PLInfo++, RefPoly++)
	{
		// �e�ӂɗאڃ|���S���͖����A�̏�Ԃɂ��Ă���
		PLInfo->LinkPolyIndex[0] = -1;
		PLInfo->LinkPolyIndex[1] = -1;
		PLInfo->LinkPolyIndex[2] = -1;

		// �אڂ���|���S����T�����߂Ƀ|���S���̐������J��Ԃ�
		FindLinkPolygon(i, PolyList, PolyLinkInfo, PLInfo, RefPoly);
	}
}

void FindLinkPolygon(int polygonIndex, MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo, POLYLINKINFO* PLInfo, MV1_REF_POLYGON* RefPoly)
{
	POLYLINKINFO* PLInfoSub;
	MV1_REF_POLYGON* RefPolySub;
	int j;

	RefPolySub = PolyList.Polygons;
	PLInfoSub = PolyLinkInfo;
	for (j = 0; j < PolyList.PolygonNum; j++, RefPolySub++, PLInfoSub++)
	{
		// �������g�̃|���S���������牽���������̃|���S����
		if (polygonIndex == j)
			continue;

		AddPolyLinkInfo(j, PLInfoSub->CenterPosition, PLInfo, RefPoly, RefPolySub);
	}
}

void AddPolyLinkInfo(int polygonIndex, VECTOR CenterPosition, POLYLINKINFO* PLInfo, MV1_REF_POLYGON* RefPoly, MV1_REF_POLYGON* RefPolySub)
{
	// �|���S���̒��_�ԍ�0��1�Ō`������ӂƗאڂ��Ă�����אڏ��ɒǉ�����
	if (PLInfo->LinkPolyIndex[0] == -1 &&
		((RefPoly->VIndex[0] == RefPolySub->VIndex[0] && RefPoly->VIndex[1] == RefPolySub->VIndex[2]) ||
			(RefPoly->VIndex[0] == RefPolySub->VIndex[1] && RefPoly->VIndex[1] == RefPolySub->VIndex[0]) ||
			(RefPoly->VIndex[0] == RefPolySub->VIndex[2] && RefPoly->VIndex[1] == RefPolySub->VIndex[1])))
	{
		PLInfo->LinkPolyIndex[0] = polygonIndex;
		PLInfo->LinkPolyDistance[0] = VSize(VSub(CenterPosition, PLInfo->CenterPosition));
	}
	else
		// �|���S���̒��_�ԍ�1��2�Ō`������ӂƗאڂ��Ă�����אڏ��ɒǉ�����
		if (PLInfo->LinkPolyIndex[1] == -1 &&
			((RefPoly->VIndex[1] == RefPolySub->VIndex[0] && RefPoly->VIndex[2] == RefPolySub->VIndex[2]) ||
				(RefPoly->VIndex[1] == RefPolySub->VIndex[1] && RefPoly->VIndex[2] == RefPolySub->VIndex[0]) ||
				(RefPoly->VIndex[1] == RefPolySub->VIndex[2] && RefPoly->VIndex[2] == RefPolySub->VIndex[1])))
		{
			PLInfo->LinkPolyIndex[1] = polygonIndex;
			PLInfo->LinkPolyDistance[1] = VSize(VSub(CenterPosition, PLInfo->CenterPosition));
		}
		else
			// �|���S���̒��_�ԍ�2��0�Ō`������ӂƗאڂ��Ă�����אڏ��ɒǉ�����
			if (PLInfo->LinkPolyIndex[2] == -1 &&
				((RefPoly->VIndex[2] == RefPolySub->VIndex[0] && RefPoly->VIndex[0] == RefPolySub->VIndex[2]) ||
					(RefPoly->VIndex[2] == RefPolySub->VIndex[1] && RefPoly->VIndex[0] == RefPolySub->VIndex[0]) ||
					(RefPoly->VIndex[2] == RefPolySub->VIndex[2] && RefPoly->VIndex[0] == RefPolySub->VIndex[1])))
			{
				PLInfo->LinkPolyIndex[2] = polygonIndex;
				PLInfo->LinkPolyDistance[2] = VSize(VSub(CenterPosition, PLInfo->CenterPosition));
			}
}

// �|���S�����m�̘A�����̌�n�����s��
void TerminatePolyLinkInfo(POLYLINKINFO* PolyLinkInfo)
{
	// �|���S�����m�̘A�������i�[���Ă����������̈�����
	free(PolyLinkInfo);
	PolyLinkInfo = NULL;
}

// �|���S�����m�̘A�������g�p���Ďw��̓�̍��W�Ԃ𒼐��I�Ɉړ��ł��邩�ǂ������`�F�b�N����( �߂�l  true:�����I�Ɉړ��ł���  false:�����I�Ɉړ��ł��Ȃ� )
bool CheckPolyMove(VECTOR StartPos, VECTOR TargetPos, POLYLINKINFO* PolyLinkInfo, MV1_REF_POLYGONLIST& PolyList)
{
	int StartPoly;
	int TargetPoly;
	POLYLINKINFO* PInfoStart;
	POLYLINKINFO* PInfoTarget;
	int i, j;
	VECTOR_D StartPosD;
	VECTOR_D TargetPosD;
	VECTOR_D PolyPos[3];
	int CheckPoly[3];
	int CheckPolyPrev[3];
	int CheckPolyNum;
	int CheckPolyPrevNum;
	int NextCheckPoly[3];
	int NextCheckPolyPrev[3];
	int NextCheckPolyNum;
	int NextCheckPolyPrevNum;

	// �J�n���W�ƖڕW���W�� y���W�l�� 0.0f �ɂ��āA���ʏ�̔���ɂ���
	StartPos.y = 0.0f;
	TargetPos.y = 0.0f;

	// ���x���グ�邽�߂� double�^�ɂ���
	StartPosD = VConvFtoD(StartPos);
	TargetPosD = VConvFtoD(TargetPos);

	// �J�n���W�ƖڕW���W�̒���A�Ⴕ���͒����ɑ��݂���|���S������������
	StartPoly = CheckOnPolyIndex(StartPos, PolyList);
	TargetPoly = CheckOnPolyIndex(TargetPos, PolyList);

	// �|���S�������݂��Ȃ�������ړ��ł��Ȃ��̂� false ��Ԃ�
	if (StartPoly == -1 || TargetPoly == -1)
		return false;

	// �J�n���W�ƖڕW���W�̒���A�Ⴕ���͒����ɑ��݂���|���S���̘A�����̃A�h���X���擾���Ă���
	PInfoStart = &PolyLinkInfo[StartPoly];
	PInfoTarget = &PolyLinkInfo[TargetPoly];

	// �w�������ɂ��邩�ǂ������`�F�b�N����|���S���Ƃ��ĊJ�n���W�̒���A�Ⴕ���͒����ɑ��݂���|���S����o�^
	CheckPolyNum = 1;
	CheckPoly[0] = StartPoly;
	CheckPolyPrevNum = 0;
	CheckPolyPrev[0] = -1;

	// ���ʂ��o��܂Ŗ������ŌJ��Ԃ�
	for (;;)
	{
		// ���̃��[�v�Ń`�F�b�N�ΏۂɂȂ�|���S���̐������Z�b�g���Ă���
		NextCheckPolyNum = 0;

		// ���̃��[�v�Ń`�F�b�N�Ώۂ���O���|���S���̐������Z�b�g���Ă���
		NextCheckPolyPrevNum = 0;

		// �`�F�b�N�Ώۂ̃|���S���̐������J��Ԃ�
		for (i = 0; i < CheckPolyNum; i++)
		{
			// �`�F�b�N�Ώۂ̃|���S���̂R���W���擾
			PolyPos[0] = VConvFtoD(PolyList.Vertexs[PolyList.Polygons[CheckPoly[i]].VIndex[0]].Position);
			PolyPos[1] = VConvFtoD(PolyList.Vertexs[PolyList.Polygons[CheckPoly[i]].VIndex[1]].Position);
			PolyPos[2] = VConvFtoD(PolyList.Vertexs[PolyList.Polygons[CheckPoly[i]].VIndex[2]].Position);

			// y���W��0.0�ɂ��āA���ʓI�Ȕ�����s���悤�ɂ���
			PolyPos[0].y = 0.0;
			PolyPos[1].y = 0.0;
			PolyPos[2].y = 0.0;

			// �|���S���̒��_�ԍ�0��1�̕ӂɗאڂ���|���S�������݂���ꍇ�ŁA
			// ���ӂ̐����ƈړ��J�n�_�A�I���_�Ō`������������ڂ��Ă����� if �����^�ɂȂ�
			if (PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[0] != -1 &&
				Segment_Segment_MinLength_SquareD(StartPosD, TargetPosD, PolyPos[0], PolyPos[1]) < 0.001)
			{
				// �����ӂƐڂ��Ă���|���S�����ڕW���W��ɑ��݂���|���S����������
				// �J�n���W����ڕW���W��܂œr�؂�Ȃ��|���S�������݂���Ƃ������ƂȂ̂� true ��Ԃ�
				if (PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[0] == TargetPoly)
					return true;

				// �ӂƐڂ��Ă���|���S�������̃`�F�b�N�Ώۂ̃|���S���ɉ�����

				// ���ɓo�^����Ă���|���S���̏ꍇ�͉����Ȃ�
				for (j = 0; j < NextCheckPolyNum; j++)
				{
					if (NextCheckPoly[j] == PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[0])
						break;
				}
				if (j == NextCheckPolyNum)
				{
					// ���̃��[�v�ŏ��O����|���S���̑Ώۂɉ�����

					// ���ɓo�^����Ă��鏜�O�|���S���̏ꍇ�͉����Ȃ�
					for (j = 0; j < NextCheckPolyPrevNum; j++)
					{
						if (NextCheckPolyPrev[j] == CheckPoly[i])
							break;
					}
					if (j == NextCheckPolyPrevNum)
					{
						NextCheckPolyPrev[NextCheckPolyPrevNum] = CheckPoly[i];
						NextCheckPolyPrevNum++;
					}

					// ��O�̃��[�v�Ń`�F�b�N�ΏۂɂȂ����|���S���̏ꍇ�������Ȃ�
					for (j = 0; j < CheckPolyPrevNum; j++)
					{
						if (CheckPolyPrev[j] == PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[0])
							break;
					}
					if (j == CheckPolyPrevNum)
					{
						// �����܂ŗ�����Q�����̃`�F�b�N�Ώۂ̃|���S���ɉ�����
						NextCheckPoly[NextCheckPolyNum] = PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[0];
						NextCheckPolyNum++;
					}
				}
			}

			// �|���S���̒��_�ԍ�1��2�̕ӂɗאڂ���|���S�������݂���ꍇ�ŁA
			// ���ӂ̐����ƈړ��J�n�_�A�I���_�Ō`������������ڂ��Ă����� if �����^�ɂȂ�
			if (PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[1] != -1 &&
				Segment_Segment_MinLength_SquareD(StartPosD, TargetPosD, PolyPos[1], PolyPos[2]) < 0.001)
			{
				// �����ӂƐڂ��Ă���|���S�����ڕW���W��ɑ��݂���|���S����������
				// �J�n���W����ڕW���W��܂œr�؂�Ȃ��|���S�������݂���Ƃ������ƂȂ̂� true ��Ԃ�
				if (PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[1] == TargetPoly)
					return true;

				// �ӂƐڂ��Ă���|���S�������̃`�F�b�N�Ώۂ̃|���S���ɉ�����

				// ���ɓo�^����Ă���|���S���̏ꍇ�͉����Ȃ�
				for (j = 0; j < NextCheckPolyNum; j++)
				{
					if (NextCheckPoly[j] == PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[1])
						break;
				}
				if (j == NextCheckPolyNum)
				{
					// ���ɓo�^����Ă��鏜�O�|���S���̏ꍇ�͉����Ȃ�
					for (j = 0; j < NextCheckPolyPrevNum; j++)
					{
						if (NextCheckPolyPrev[j] == CheckPoly[i])
							break;
					}
					if (j == NextCheckPolyPrevNum)
					{
						NextCheckPolyPrev[NextCheckPolyPrevNum] = CheckPoly[i];
						NextCheckPolyPrevNum++;
					}

					// ��O�̃��[�v�Ń`�F�b�N�ΏۂɂȂ����|���S���̏ꍇ�������Ȃ�
					for (j = 0; j < CheckPolyPrevNum; j++)
					{
						if (CheckPolyPrev[j] == PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[1])
							break;
					}
					if (j == CheckPolyPrevNum)
					{
						// �����܂ŗ�����Q�����̃`�F�b�N�Ώۂ̃|���S���ɉ�����
						NextCheckPoly[NextCheckPolyNum] = PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[1];
						NextCheckPolyNum++;
					}
				}
			}

			// �|���S���̒��_�ԍ�2��0�̕ӂɗאڂ���|���S�������݂���ꍇ�ŁA
			// ���ӂ̐����ƈړ��J�n�_�A�I���_�Ō`������������ڂ��Ă����� if �����^�ɂȂ�
			if (PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[2] != -1 &&
				Segment_Segment_MinLength_SquareD(StartPosD, TargetPosD, PolyPos[2], PolyPos[0]) < 0.001)
			{
				// �����ӂƐڂ��Ă���|���S�����ڕW���W��ɑ��݂���|���S����������
				// �J�n���W����ڕW���W��܂œr�؂�Ȃ��|���S�������݂���Ƃ������ƂȂ̂� true ��Ԃ�
				if (PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[2] == TargetPoly)
					return true;

				// �ӂƐڂ��Ă���|���S�������̃`�F�b�N�Ώۂ̃|���S���ɉ�����

				// ���ɓo�^����Ă���|���S���̏ꍇ�͉����Ȃ�
				for (j = 0; j < NextCheckPolyNum; j++)
				{
					if (NextCheckPoly[j] == PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[2])
						break;
				}
				if (j == NextCheckPolyNum)
				{
					// ���ɓo�^����Ă��鏜�O�|���S���̏ꍇ�͉����Ȃ�
					for (j = 0; j < NextCheckPolyPrevNum; j++)
					{
						if (NextCheckPolyPrev[j] == CheckPoly[i])
							break;
					}
					if (j == NextCheckPolyPrevNum)
					{
						NextCheckPolyPrev[NextCheckPolyPrevNum] = CheckPoly[i];
						NextCheckPolyPrevNum++;
					}

					// ��O�̃��[�v�Ń`�F�b�N�ΏۂɂȂ����|���S���̏ꍇ�������Ȃ�
					for (j = 0; j < CheckPolyPrevNum; j++)
					{
						if (CheckPolyPrev[j] == PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[2])
							break;
					}
					if (j == CheckPolyPrevNum)
					{
						// �����܂ŗ�����Q�����̃`�F�b�N�Ώۂ̃|���S���ɉ�����
						NextCheckPoly[NextCheckPolyNum] = PolyLinkInfo[CheckPoly[i]].LinkPolyIndex[2];
						NextCheckPolyNum++;
					}
				}
			}
		}

		// ���̃��[�v�Ń`�F�b�N�ΏۂɂȂ�|���S��������Ȃ������Ƃ������Ƃ�
		// �ړ��J�n�_�A�I���_�Ō`����������Ɛڂ���`�F�b�N�Ώۂ̃|���S���ɗאڂ���
		// �|���S��������Ȃ������Ƃ������ƂȂ̂ŁA�����I�Ȉړ��͂ł��Ȃ��Ƃ������Ƃ� false ��Ԃ�
		if (NextCheckPolyNum == 0)
		{
			return false;
		}

		// ���Ƀ`�F�b�N�ΏۂƂȂ�|���S���̏����R�s�[����
		for (i = 0; i < NextCheckPolyNum; i++)
		{
			CheckPoly[i] = NextCheckPoly[i];
		}
		CheckPolyNum = NextCheckPolyNum;

		// ���Ƀ`�F�b�N�ΏۊO�ƂȂ�|���S���̏����R�s�[����
		for (i = 0; i < NextCheckPolyPrevNum; i++)
		{
			CheckPolyPrev[i] = NextCheckPolyPrev[i];
		}
		CheckPolyPrevNum = NextCheckPolyPrevNum;
	}
}

// �|���S�����m�̘A�������g�p���Ďw��̓�̍��W�Ԃ𒼐��I�Ɉړ��ł��邩�ǂ������`�F�b�N����( �߂�l  true:�����I�Ɉړ��ł���  false:�����I�Ɉړ��ł��Ȃ� )( ���w��� )
bool CheckPolyMoveWidth(VECTOR StartPos, VECTOR TargetPos, float Width, POLYLINKINFO* PolyLinkInfo, MV1_REF_POLYGONLIST& PolyList)
{
	VECTOR Direction;
	VECTOR SideDirection;
	VECTOR TempVec;

	// �ŏ��ɊJ�n���W����ڕW���W�ɒ����I�Ɉړ��ł��邩�ǂ������`�F�b�N
	if (CheckPolyMove(StartPos, TargetPos, PolyLinkInfo, PolyList) == false)
		return false;

	// �J�n���W����ڕW���W�Ɍ������x�N�g�����Z�o
	Direction = VSub(TargetPos, StartPos);

	// y���W�� 0.0f �ɂ��ĕ��ʓI�ȃx�N�g���ɂ���
	Direction.y = 0.0f;

	// �J�n���W����ڕW���W�Ɍ������x�N�g���ɒ��p�Ȑ��K���x�N�g�����Z�o
	SideDirection = VCross(Direction, VGet(0.0f, 1.0f, 0.0f));
	SideDirection = VNorm(SideDirection);

	// �J�n���W�ƖڕW���W�� Width / 2.0f ���������������ɂ��炵�āA�ēx�����I�Ɉړ��ł��邩�ǂ������`�F�b�N
	TempVec = VScale(SideDirection, Width / 2.0f);
	if (CheckPolyMove(VAdd(StartPos, TempVec), VAdd(TargetPos, TempVec), PolyLinkInfo, PolyList) == false)
		return false;

	// �J�n���W�ƖڕW���W�� Width / 2.0f ��������O�Ƃ͋t�����̐��������ɂ��炵�āA�ēx�����I�Ɉړ��ł��邩�ǂ������`�F�b�N
	TempVec = VScale(SideDirection, -Width / 2.0f);
	if (CheckPolyMove(VAdd(StartPos, TempVec), VAdd(TargetPos, TempVec), PolyLinkInfo, PolyList) == false)
		return false;

	// �����܂ł�����w��̕��������Ă������I�Ɉړ��ł���Ƃ������ƂȂ̂� true ��Ԃ�
	return true;
}


// �]���}�b�v�쐬�p�̒T��
void MapSearch(VECTOR PositionList[4], PATHPLANNING PathPlanning[4], PATHPLANNING influenceMap[3], MV1_REF_POLYGONLIST& PolyList, POLYLINKINFO* PolyLinkInfo)
{
	int i, j, k;
	int PolyIndex;
	PATHPLANNING_UNIT* PUnit;
	PATHPLANNING_UNIT* PUnitSub;
	PATHPLANNING_UNIT* PUnitSub2;

	float maxDistance = 0;

	for (j = 0; j < 4; j++)
	{
		maxDistance = 0;

		// �o�H�T���p�̃|���S������������
		PUnit = PathPlanning[j].UnitArray;
		for (i = 0; i < PolyList.PolygonNum; i++, PUnit++)
		{
			PUnit->PolyIndex = i;
			PUnit->TotalDistance = 0.0f;
			PUnit->PrevPolyIndex = -1;
			PUnit->NextPolyIndex = -1;
			PUnit->ActiveNextUnit = NULL;
		}

		PolyIndex = CheckOnPolyIndex(PositionList[j], PolyList);
		if (PolyIndex == -1)
			continue;
		PathPlanning[j].StartUnit = &PathPlanning[j].UnitArray[PolyIndex];

		// �o�H�T�������Ώۂ̃|���S���Ƃ��ăX�^�[�g�n�_�ɂ���|���S����o�^����
		PathPlanning[j].ActiveFirstUnit = &PathPlanning[j].UnitArray[PolyIndex];

		// �o�H��T�����ăS�[���n�_�̃|���S���ɂ��ǂ蒅���܂Ń��[�v���J��Ԃ�
		while (PathPlanning[j].ActiveFirstUnit != NULL)
		{
			// �o�H�T�������ΏۂɂȂ��Ă���|���S�������ׂď���
			PUnit = PathPlanning[j].ActiveFirstUnit;
			PathPlanning[j].ActiveFirstUnit = NULL;
			for (; PUnit != NULL; PUnit = PUnit->ActiveNextUnit)
			{
				// �|���S���̕ӂ̐������J��Ԃ�
				for (i = 0; i < 3; i++)
				{
					// �ӂɗאڂ���|���S���������ꍇ�͉������Ȃ�
					if (PolyLinkInfo[PUnit->PolyIndex].LinkPolyIndex[i] == -1)
						continue;

					// �אڂ���|���S�������Ɍo�H�T���������s���Ă��āA����苗���̒����o�H�ƂȂ��Ă��邩�A
					// �X�^�[�g�n�_�̃|���S���������ꍇ�͉������Ȃ�
					PUnitSub = &PathPlanning[j].UnitArray[PolyLinkInfo[PUnit->PolyIndex].LinkPolyIndex[i]];
					if ((PUnitSub->PrevPolyIndex != -1 && PUnitSub->TotalDistance <= PUnit->TotalDistance + PolyLinkInfo[PUnit->PolyIndex].LinkPolyDistance[i])
						|| PUnitSub->PolyIndex == PathPlanning[j].StartUnit->PolyIndex)
						continue;

					// �אڂ���|���S���Ɍo�H���ƂȂ鎩���̃|���S���̔ԍ���������
					PUnitSub->PrevPolyIndex = PUnit->PolyIndex;

					// �אڂ���|���S���ɂ����ɓ��B����܂ł̋�����������
					PUnitSub->TotalDistance = PUnit->TotalDistance + PolyLinkInfo[PUnit->PolyIndex].LinkPolyDistance[i];

					if (maxDistance < PUnitSub->TotalDistance)
					{
						maxDistance = PUnitSub->TotalDistance;
					}

					// ���̃��[�v�ōs���o�H�T�������Ώۂɒǉ�����A���ɒǉ�����Ă�����ǉ����Ȃ�
					for (PUnitSub2 = PathPlanning[j].ActiveFirstUnit; PUnitSub2 != NULL; PUnitSub2 = PUnitSub2->ActiveNextUnit)
					{
						if (PUnitSub2 == PUnitSub)
							break;
					}
					if (PUnitSub2 == NULL)
					{
						PUnitSub->ActiveNextUnit = PathPlanning[j].ActiveFirstUnit;
						PathPlanning[j].ActiveFirstUnit = PUnitSub;
					}
				}
			}

		}

		// �]���l���v�Z
		PUnit = PathPlanning[j].UnitArray;
		for (int i = 0; i < PolyList.PolygonNum; i++, PUnit++)
		{
			PUnit->NormDistance = 1.0 - PUnit->TotalDistance / maxDistance;
		}
	}


	for (j = 0; j < 3; j++)
	{
		// �o�H�T���p�̃|���S������������
		PUnit = influenceMap[j].UnitArray;
		for (i = 0; i < PolyList.PolygonNum; i++, PUnit++)
		{
			PUnit->PolyIndex = i;
			PUnit->NormDistance = -1.0f;
		}
	}

	for (i = 0; i < 3; i++)
	{
		for (k = 0; k < 4; k++)
		{
			if (k == i + 1)
				continue;

			PUnit = influenceMap[i].UnitArray;

			PUnitSub = PathPlanning[k].UnitArray;
			for (j = 0; j < PolyList.PolygonNum; j++, PUnit++, PUnitSub++)
			{
				if (PUnit->NormDistance < PUnitSub->NormDistance)
					PUnit->NormDistance = PUnitSub->NormDistance;
			}
		}
	}
}

void DrawEvaluationMap(PATHPLANNING& PathPlanning, MV1_REF_POLYGONLIST& PolyList)
{
	PATHPLANNING_UNIT* PUnit;

	// �T�������o�H�̃|���S���̗֊s��`�悷��( �f�o�b�O�\�� )
	PUnit = PathPlanning.UnitArray;
	for (int i = 0; i < PolyList.PolygonNum; i++, PUnit++)
	{
		int polygonColor = 255 * PUnit->NormDistance;

		DrawTriangle3D(
			PolyList.Vertexs[PolyList.Polygons[PUnit->PolyIndex].VIndex[0]].Position,
			PolyList.Vertexs[PolyList.Polygons[PUnit->PolyIndex].VIndex[1]].Position,
			PolyList.Vertexs[PolyList.Polygons[PUnit->PolyIndex].VIndex[2]].Position,
			GetColor(150, 255, 150),
			TRUE
		);
	}
}

// �w��̂Q�_�̌o�H��T������( �߂�l  true:�o�H�\�z����  false:�o�H�\�z���s( �X�^�[�g�n�_�ƃS�[���n�_���q���o�H������������ ) )
bool SetupPathPlanning(VECTOR StartPos, VECTOR GoalPos, MV1_REF_POLYGONLIST& PolyList, PATHPLANNING& PathPlanning, POLYLINKINFO* PolyLinkInfo)
{
	int i;
	int PolyIndex;
	PATHPLANNING_UNIT* PUnit;
	PATHPLANNING_UNIT* PUnitSub;
	PATHPLANNING_UNIT* PUnitSub2;
	bool Goal;

	// �X�^�[�g�ʒu�ƃS�[���ʒu��ۑ�
	PathPlanning.StartPosition = StartPos;
	PathPlanning.GoalPosition = GoalPos;

	// �o�H�T���p�̃|���S�������i�[���郁�����̈���m�ۂ���
	PathPlanning.UnitArray = (PATHPLANNING_UNIT*)malloc(sizeof(PATHPLANNING_UNIT) * PolyList.PolygonNum);

	// �o�H�T���p�̃|���S������������
	PUnit = PathPlanning.UnitArray;
	for (i = 0; i < PolyList.PolygonNum; i++, PUnit++)
	{
		PUnit->PolyIndex = i;
		PUnit->TotalDistance = 0.0f;
		PUnit->PrevPolyIndex = -1;
		PUnit->NextPolyIndex = -1;
		PUnit->ActiveNextUnit = NULL;
	}

	// �X�^�[�g�n�_�ɂ���|���S���̔ԍ����擾���A�|���S���̌o�H�T�������p�̍\���̂̃A�h���X��ۑ�
	PolyIndex = CheckOnPolyIndex(StartPos, PolyList);
	if (PolyIndex == -1)
		return false;
	PathPlanning.StartUnit = &PathPlanning.UnitArray[PolyIndex];

	// �o�H�T�������Ώۂ̃|���S���Ƃ��ăX�^�[�g�n�_�ɂ���|���S����o�^����
	PathPlanning.ActiveFirstUnit = &PathPlanning.UnitArray[PolyIndex];

	// �S�[���n�_�ɂ���|���S���̔ԍ����擾���A�|���S���̌o�H�T�������p�̍\���̂̃A�h���X��ۑ�
	PolyIndex = CheckOnPolyIndex(GoalPos, PolyList);
	if (PolyIndex == -1)
		return false;
	PathPlanning.GoalUnit = &PathPlanning.UnitArray[PolyIndex];

	// �S�[���n�_�ɂ���|���S���ƃX�^�[�g�n�_�ɂ���|���S���������������� false ��Ԃ�
	if (PathPlanning.GoalUnit == PathPlanning.StartUnit)
		return false;

	// �o�H��T�����ăS�[���n�_�̃|���S���ɂ��ǂ蒅���܂Ń��[�v���J��Ԃ�
	Goal = false;
	while (Goal == false)
	{
		// �o�H�T�������ΏۂɂȂ��Ă���|���S�������ׂď���
		PUnit = PathPlanning.ActiveFirstUnit;
		PathPlanning.ActiveFirstUnit = NULL;
		for (; PUnit != NULL; PUnit = PUnit->ActiveNextUnit)
		{
			// �|���S���̕ӂ̐������J��Ԃ�
			for (i = 0; i < 3; i++)
			{
				// �ӂɗאڂ���|���S���������ꍇ�͉������Ȃ�
				if (PolyLinkInfo[PUnit->PolyIndex].LinkPolyIndex[i] == -1)
					continue;

				// �אڂ���|���S�������Ɍo�H�T���������s���Ă��āA����苗���̒����o�H�ƂȂ��Ă��邩�A
				// �X�^�[�g�n�_�̃|���S���������ꍇ�͉������Ȃ�
				PUnitSub = &PathPlanning.UnitArray[PolyLinkInfo[PUnit->PolyIndex].LinkPolyIndex[i]];
				if ((PUnitSub->PrevPolyIndex != -1 && PUnitSub->TotalDistance <= PUnit->TotalDistance + PolyLinkInfo[PUnit->PolyIndex].LinkPolyDistance[i])
					|| PUnitSub->PolyIndex == PathPlanning.StartUnit->PolyIndex)
					continue;

				// �אڂ���|���S�����S�[���n�_�ɂ���|���S����������S�[���ɒH�蒅�����t���O�𗧂Ă�
				if (PUnitSub->PolyIndex == PathPlanning.GoalUnit->PolyIndex)
				{
					Goal = true;
				}

				// �אڂ���|���S���Ɍo�H���ƂȂ鎩���̃|���S���̔ԍ���������
				PUnitSub->PrevPolyIndex = PUnit->PolyIndex;

				// �אڂ���|���S���ɂ����ɓ��B����܂ł̋�����������
				PUnitSub->TotalDistance = PUnit->TotalDistance + PolyLinkInfo[PUnit->PolyIndex].LinkPolyDistance[i];

				// ���̃��[�v�ōs���o�H�T�������Ώۂɒǉ�����A���ɒǉ�����Ă�����ǉ����Ȃ�
				for (PUnitSub2 = PathPlanning.ActiveFirstUnit; PUnitSub2 != NULL; PUnitSub2 = PUnitSub2->ActiveNextUnit)
				{
					if (PUnitSub2 == PUnitSub)
						break;
				}
				if (PUnitSub2 == NULL)
				{
					PUnitSub->ActiveNextUnit = PathPlanning.ActiveFirstUnit;
					PathPlanning.ActiveFirstUnit = PUnitSub;
				}
			}
		}

		// �����ɂ������� PathPlanning.ActiveFirstUnit �� NULL �Ƃ������Ƃ�
		// �X�^�[�g�n�_�ɂ���|���S������S�[���n�_�ɂ���|���S���ɒH�蒅���Ȃ��Ƃ������ƂȂ̂� false ��Ԃ�
		if (PathPlanning.ActiveFirstUnit == NULL)
		{
			return false;
		}
	}

	// �S�[���n�_�̃|���S������X�^�[�g�n�_�̃|���S���ɒH����
	// �o�H��̃|���S���Ɏ��Ɉړ����ׂ��|���S���̔ԍ���������
	PUnit = PathPlanning.GoalUnit;
	do
	{
		PUnitSub = PUnit;

		// �S�[���n�_�̃|���S����PrevPolyIndex�̔ԍ��ɑΉ������|���S�����Q�Ƃ����
		PUnit = &PathPlanning.UnitArray[PUnitSub->PrevPolyIndex];

		// �S�[���n�_�̃|���S���̃C���f�b�N�X�ԍ�����
		PUnit->NextPolyIndex = PUnitSub->PolyIndex;

	} while (PUnit != PathPlanning.StartUnit);

	// �����ɂ�����X�^�[�g�n�_����S�[���n�_�܂ł̌o�H���T���ł����Ƃ������ƂȂ̂� true ��Ԃ�
	return true;
}

// �o�H�T�����̌�n��
void TerminatePathPlanning(PATHPLANNING PathPlanning[4])
{
	for (int i = 0; i < 4; i++)
	{
		// �o�H�T���ׂ̈Ɋm�ۂ����������̈�����
		free(PathPlanning[i].UnitArray);
		PathPlanning[i].UnitArray = NULL;
	}
}

// �T�������o�H���ړ����鏈���̏��������s���֐�
void MoveInitialize(bool isEscale[3], VECTOR PositionList[4], MV1_REF_POLYGONLIST& PolyList, PATHMOVEINFO PathMove[3], PATHPLANNING influenceMap[3], POLYLINKINFO* PolyLinkInfo)
{

	for (int i = 0; i < 3; i++)
	{
		// �ړ��J�n���_�ŏ���Ă���|���S���̓X�^�[�g�n�_�ɂ���|���S��
		PathMove[i].NowPolyIndex = CheckOnPolyIndex(PositionList[i + 1], PolyList);

		// �ړ��J�n���_�̍��W�̓X�^�[�g�n�_�ɂ���|���S���̒��S���W
		PositionList[i + 1] = PathMove[i].NowPosition = PolyLinkInfo[PathMove[i].NowPolyIndex].CenterPosition;

		RefreshMoveDirectionMap(i + 1, isEscale[i], PathMove[i], influenceMap[i], PolyLinkInfo);
	}
}

// �T�������o�H���ړ����鏈���̂P�t���[�����̏������s���֐�
void MoveProcess(bool isEscale[3], VECTOR PositionList[4], MV1_REF_POLYGONLIST& PolyList, PATHMOVEINFO PathMove[3], PATHPLANNING influenceMap[3], POLYLINKINFO* PolyLinkInfo)
{

	for (int i = 0; i < 3; i++)
	{
		if (PathMove[i].TargetPathPlanningUnit->PolyIndex == PathMove[i].NowPolyIndex)
			RefreshMoveDirectionMap(i + 1, isEscale[i], PathMove[i], influenceMap[i], PolyLinkInfo);

		// �ړ������ɍ��W���ړ�����
		PositionList[i + 1] = PathMove[i].NowPosition = VAdd(PathMove[i].NowPosition, VScale(PathMove[i].MoveDirection, MOVESPEED));

		// ���݂̍��W�ŏ���Ă���|���S������������
		PathMove[i].NowPolyIndex = CheckOnPolyIndex(PathMove[i].NowPosition, PolyList);
	}
}

// �T�������o�H���ړ����鏈���ňړ��������X�V���鏈�����s���֐�
void RefreshMoveDirectionMap(int index, bool isEscape, PATHMOVEINFO& PathMove, PATHPLANNING influenceMap, POLYLINKINFO* PolyLinkInfo)
{
	float evaluationValue = 0.0;
	int linkPolygonIndex = 0;

	evaluationValue = influenceMap.UnitArray[PathMove.NowPolyIndex].NormDistance;
	PathMove.TargetPathPlanningUnit = &influenceMap.UnitArray[PathMove.NowPolyIndex];

	int i, j, k;

	// ���݂̈ʒu�̃|���S���ɗאڂ��Ă���|���S����T��
	for (j = 0; j < 3; j++)
	{
		// �ӂɗאڂ���|���S���������ꍇ�͉������Ȃ�
		if (PolyLinkInfo[PathMove.NowPolyIndex].LinkPolyIndex[j] == -1)
			continue;

		linkPolygonIndex = PolyLinkInfo[PathMove.NowPolyIndex].LinkPolyIndex[j];

		// ����ɗאڂ���|���S����T��
		for (k = 0; k < 3; k++)
		{
			if (PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k] == -1 ||
				PathMove.NowPolyIndex == PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k])
				continue;

			if (isEscape)
			{
				if (evaluationValue > influenceMap.UnitArray[PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k]].NormDistance)
				{
					evaluationValue = influenceMap.UnitArray[PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k]].NormDistance;
					PathMove.TargetPathPlanningUnit = &influenceMap.UnitArray[PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k]];
				}
			}
			else
			{
				if (evaluationValue < influenceMap.UnitArray[PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k]].NormDistance)
				{
					evaluationValue = influenceMap.UnitArray[PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k]].NormDistance;
					PathMove.TargetPathPlanningUnit = &influenceMap.UnitArray[PolyLinkInfo[linkPolygonIndex].LinkPolyIndex[k]];
				}
			}
		}
	}


	// �ړ����������肷��A�ړ������͌��݂̍��W���璆�Ԓn�_�̃|���S���̒��S���W�Ɍ���������
	PathMove.MoveDirection = VSub(PolyLinkInfo[PathMove.TargetPathPlanningUnit->PolyIndex].CenterPosition, PathMove.NowPosition);
	PathMove.MoveDirection.y = 0.0f;
	PathMove.MoveDirection = VNorm(PathMove.MoveDirection);
}

// �T�������o�H���ړ����鏈���ňړ��������X�V���鏈�����s���֐�( �߂�l  true:�S�[���ɒH�蒅���Ă���  false:�S�[���ɒH�蒅���Ă��Ȃ� )
bool RefreshMoveDirection(PATHMOVEINFO& PathMove, PATHPLANNING& PathPlanning, POLYLINKINFO* PolyLinkInfo, MV1_REF_POLYGONLIST& PolyList)
{
	PATHPLANNING_UNIT* TempPUnit;

	// ���ݏ���Ă���|���S�����S�[���n�_�ɂ���|���S���̏ꍇ�͏����𕪊�
	if (PathMove.NowPathPlanningUnit == PathPlanning.GoalUnit)
	{
		// �����͖ڕW���W
		PathMove.MoveDirection = VSub(PathPlanning.GoalPosition, PathMove.NowPosition);
		PathMove.MoveDirection.y = 0.0f;

		// �ڕW���W�܂ł̋������ړ����x�ȉ���������S�[���ɒH��������Ƃɂ���
		if (VSize(PathMove.MoveDirection) <= MOVESPEED)
		{
			return true;
		}

		// ����ȊO�̏ꍇ�͂܂����ǂ蒅���Ă��Ȃ����̂Ƃ��Ĉړ�����
		PathMove.MoveDirection = VNorm(PathMove.MoveDirection);

		return false;
	}

	// ���ݏ���Ă���|���S�����ړ����Ԓn�_�̃|���S���̏ꍇ�͎��̒��Ԓn�_�����肷�鏈�����s��
	if (PathMove.NowPathPlanningUnit == PathMove.TargetPathPlanningUnit)
	{
		// ���̒��Ԓn�_�����肷��܂Ń��[�v��������
		for (;;)
		{
			TempPUnit = &PathPlanning.UnitArray[PathMove.TargetPathPlanningUnit->NextPolyIndex];

			// �o�H��̎��̃|���S���̒��S���W�ɒ����I�Ɉړ��ł��Ȃ��ꍇ�̓��[�v���甲����
			if (CheckPolyMoveWidth(PathMove.NowPosition, PolyLinkInfo[TempPUnit->PolyIndex].CenterPosition, COLLWIDTH, PolyLinkInfo, PolyList) == false)
				break;

			// �`�F�b�N�Ώۂ��o�H��̍X�Ɉ��̃|���S���ɕύX����
			PathMove.TargetPathPlanningUnit = TempPUnit;

			// �����S�[���n�_�̃|���S���������烋�[�v�𔲂���
			if (PathMove.TargetPathPlanningUnit == PathPlanning.GoalUnit)
				break;
		}
	}

	// �ړ����������肷��A�ړ������͌��݂̍��W���璆�Ԓn�_�̃|���S���̒��S���W�Ɍ���������
	PathMove.MoveDirection = VSub(PolyLinkInfo[PathMove.TargetPathPlanningUnit->PolyIndex].CenterPosition, PathMove.NowPosition);
	PathMove.MoveDirection.y = 0.0f;
	PathMove.MoveDirection = VNorm(PathMove.MoveDirection);

	// �����ɗ����Ƃ������Ƃ̓S�[���ɒH�蒅���Ă��Ȃ��̂� false ��Ԃ�
	return false;
}

void SetActionFlag(bool isEscape[3])
{
	for (int i = 0; i < 3; i++)
	{
		isEscape[i] = GetRand(1);
	}
}

// WinMain �֐�
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	PATHPLANNING_UNIT* PUnit;

	// �E�C���h�E���[�h�ŋN��
	ChangeWindowMode(TRUE);

	// �c�w���C�u�����̏�����
	if (DxLib_Init() < 0) return -1;

	VECTOR position[4] = { VGet(-7400.0f, 0.0f, -7400.0f),
							VGet(-7400.0f, 0.0f,  7400.0f),
							VGet(7400.0f, 0.0f,  7400.0f),
							VGet(7400.0f, 0.0f, -7400.0f) };

	int StageModelHandle;			// �X�e�[�W���f���n���h��
	MV1_REF_POLYGONLIST PolyList;	// �X�e�[�W���f���̃|���S�����

	POLYLINKINFO* PolyLinkInfo = nullptr;		// �X�e�[�W���f���̑S�|���S�����́u�|���S�����m�̘A�����v�̔z�񂪊i�[���i�[���ꂽ�������G���A�̐擪�A�h���X���i�[����ϐ�

	PATHPLANNING PathPlanning[4];		// �o�H�T�������p�̍\����
	PATHPLANNING influenceMap[3];		// �o�H�T�������p�̍\����
	PATHMOVEINFO PathMove[3];			// �T�������o�H���ړ����鏈���Ɏg�p�������Z�߂��\����

	float maxDistance = 0;

	bool isEscape[3] = { false };

	int i, j;

	int frame = 0;

	// �X�e�[�W���f���̓ǂݍ���
	StageModelHandle = MV1LoadModel("stage.mv1");

		// �X�e�[�W���f���S�̂̎Q�Ɨp���b�V�����\�z����
	MV1SetupReferenceMesh(StageModelHandle, 0, TRUE);

	// �X�e�[�W���f���S�̂̎Q�Ɨp���b�V���̏����擾����
	PolyList = MV1GetReferenceMesh(StageModelHandle, 0, TRUE);

	// �X�e�[�W���f���̑S�|���S���̘A�������i�[����ׂ̃������̈���m�ۂ���
	PolyLinkInfo = (POLYLINKINFO*)malloc(sizeof(POLYLINKINFO) * PolyList.PolygonNum);

	// �X�e�[�W���f���̃|���S�����m�̘A�������\�z����
	SetupPolyLinkInfo(StageModelHandle, PolyList, PolyLinkInfo);

	// �`���𗠉�ʂɂ���
	SetDrawScreen(DX_SCREEN_BACK);

	SetLightDirection(VGet(0.0f, 0.1f, 0.0f));

	for (i = 0; i < 4; i++)
	{
		// �o�H�T���p�̃|���S�������i�[���郁�����̈���m�ۂ���
		PathPlanning[i].UnitArray = (PATHPLANNING_UNIT*)malloc(sizeof(PATHPLANNING_UNIT) * PolyList.PolygonNum);
	}

	for (i = 0; i < 3; i++)
	{
		// �o�H�T���p�̃|���S�������i�[���郁�����̈���m�ۂ���
		influenceMap[i].UnitArray = (PATHPLANNING_UNIT*)malloc(sizeof(PATHPLANNING_UNIT) * PolyList.PolygonNum);
	}

	SetActionFlag(isEscape);

	MapSearch(position, PathPlanning, influenceMap, PolyList, PolyLinkInfo);

	//// �T�������o�H����ړ����鏀�����s��
	MoveInitialize(isEscape, position, PolyList, PathMove, influenceMap, PolyLinkInfo);

	VECTOR drawPos = VGet(0, 0, 0);

	bool isPush = false;

	bool isStart = false;

	// ���C�����[�v(�����L�[�������ꂽ�烋�[�v�𔲂���)
	while (ProcessMessage() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0)
	{
		frame++;

		// ��ʂ̏�����
		ClearDrawScreen();

		clsDx();

		if (CheckHitKey(KEY_INPUT_LEFT) == 1)
			position[0] = VAdd(position[0], VScale(VGet(-1, 0, 0), 25));

		if (CheckHitKey(KEY_INPUT_RIGHT) == 1)
			position[0] = VAdd(position[0], VScale(VGet(1, 0, 0), 25));

		if (CheckHitKey(KEY_INPUT_UP) == 1)
			position[0] = VAdd(position[0], VScale(VGet(0, 0, 1), 25));

		if (CheckHitKey(KEY_INPUT_DOWN) == 1)
			position[0] = VAdd(position[0], VScale(VGet(0, 0, -1), 25));

		if (frame % 300 == 0)
		{
			SetActionFlag(isEscape);
		}

		MapSearch(position, PathPlanning, influenceMap, PolyList, PolyLinkInfo);

		MoveProcess(isEscape, position, PolyList, PathMove, influenceMap, PolyLinkInfo);

		// �J�����̐ݒ�
		SetCameraPositionAndTarget_UpVecY(VGet(-0.000f, 15692.565f, -3121.444f), VGet(0.000f, 0.000f, 0.000f));
		SetCameraNearFar(320.000f, 80000.000f);

		// �X�e�[�W���f����`�悷��
		MV1DrawModel(StageModelHandle);

		DrawEvaluationMap(influenceMap[0], PolyList);

		// �ړ����̌��ݍ��W�ɋ��̂�`�悷��

		DrawSphere3D(VAdd(position[0], VGet(0.0f, 100.0f, 0.0f)), SPHERESIZE, 16, GetColor(255, 0, 0), GetColor(255, 255, 255), TRUE);

		for (i = 1; i < 4; i++)
		{
			DrawSphere3D(VAdd(position[i], VGet(0.0f, 100.0f, 0.0f)), SPHERESIZE, 16, GetColor(0, 0, 255), GetColor(255, 255, 255), TRUE);
		}

		printfDx("%d", frame);

		// ����ʂ̓��e��\��ʂɔ��f
		ScreenFlip();
	}

	// �o�H���̌�n��
	TerminatePathPlanning(PathPlanning);

	// �X�e�[�W���f���̃|���S�����m�̘A�����̌�n��
	TerminatePolyLinkInfo(PolyLinkInfo);

	// �c�w���C�u�����̌�n��
	DxLib_End();

	// �\�t�g�̏I��
	return 0;
}
