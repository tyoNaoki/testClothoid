// Fill out your copyright notice in the Description page of Project Settings.


#include "clothoidLine.h"
#include "Components/BillboardComponent.h"
#include <iostream>
#include "Kismet/KismetMathLibrary.h"

using namespace std;

// Sets default values
AclothoidLine::AclothoidLine()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	billboards.Empty();

	for(auto&x : locs){

		UBillboardComponent* billboard = CreateDefaultSubobject<UBillboardComponent>(TEXT("BillBoard"));
		billboards.Add(billboard);
	}

	pointMark = CreateDefaultSubobject<UBillboardComponent>(TEXT("BillBoard"));
}

// Called when the game starts or when spawned
void AclothoidLine::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AclothoidLine::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AclothoidLine::simpson_integral(FSlope f, float a, float b, std::complex<float>* r) {
	float mul = (b - a) * static_cast<float>(1.0 / 6.0);
	*r = mul * (f(a) + static_cast<float>(4.0) * f((a + b) * static_cast<float>(0.5)) + f(b));
};

void AclothoidLine::phiSimpson_integral(FPhiSlope f, float a, float b, std::complex<float>* r) {
	float mul = (b - a) * static_cast<float>(1.0 / 6.0);
	*r = mul * (f(a) + static_cast<float>(4.0) * f((a + b) * static_cast<float>(0.5)) + f(b));
};

/*
void AclothoidLine::CalcClothoidSplineResults(){

	const int32 Value1 = 10;
	//��`
	TFunctionRef<float(const int32)> TestLambda = [&Value1](const int32 Value2)
	{
		return static_cast<float>(Value1 + Value2);
	};
	//�Ăяo��
	//float Result = TestLambda(1);

	TFunctionRef<float(const int32)> solve = [&Value1](const int32 x) {
	
		int32 y1 = 0;
		int32 y2 = 0;
		int32 x1 = 0;
		int32 x2 = 0;
		int32 diff_x = 0;
		int32 next_x = 0;
	
		x1 = x;
	
		//while (1) {
		//
		//	x2 = x1 + delta;
		//
		//	y1 = func(x1);
		//	y2 = func(x2);
		//
		//	diff_x = (y2 - y1) / (x2 - x1);
		//	next_x = x1 - y1 / diff_x;
		//
		//	if (fabs(y1 - 0) < error) {
		//		break;
		//	}
		//	x1 = next_x;
		//}
	
		return static_cast<float>(x1);
	};

	if(locs.Num()<=1){
		return;
	}

	points.Empty();

	FVector startLoc = (pointMark->GetComponentLocation());
	startLoc += FVector(locs[0].X,locs[0].Y,0.0f);
	//FVector2D tangentLoc = locs[1] - locs[0];

	float s = 0.5f;

	float h = _hValue;
	//float h = tangentLoc.X;
	//float l_x = tangentLoc.X;
	//float l_y = tangentLoc.Y;

	
	float phi0 = UKismetMathLibrary::DegreesToRadians(_phi0Value);
	float phiV = _phiVValue;
	float phiU = _phiUValue;

	float stepS = 1.0f / n;

	FSlope slope;
	slope.phi0 = phi0;
	slope.phiV = phiV;
	slope.phiU = phiU;

	FRotator Rotation = UKismetMathLibrary::FindLookAtRotation(startLoc, FVector(locs[1].X, locs[1].Y, startLoc.Z));
	FRotator YawRotation(0, Rotation.Yaw, 0);

	//// get forward vector
	FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);

	std::complex<float> P_Vector;
	for (int i = 0; i < n; ++i) {
		float S = stepS * i;
	
		std::complex<float> r;
		simpson_integral(slope, S, S + stepS, &r);
		P_Vector += r;

		float x = P_Vector.real();
		float y = P_Vector.imag();

		float differ = (float)(i+1)/n;
		//UE_LOG(LogTemp, Log, TEXT("differ = %d"),differ);

		FVector2D point2D = (h * FVector2D(x, y));
		//FVector2D point2D = FVector2D(x*l_x,y*l_y);
		
		//FVector clothoidCurveLoc = FVector(point2D.X + startLoc.X + (tangentLoc.X * differ), point2D.Y + startLoc.Y + (tangentLoc.Y * differ), startLoc.Z - 35.0f);
		FVector clothoidCurveLoc = FVector(point2D.X + startLoc.X, point2D.Y + startLoc.Y, startLoc.Z - 35.0f);
		points.Add(clothoidCurveLoc);

		if(i==n-1){
			UE_LOG(LogTemp,Log,TEXT("@ = %f"), FSlope().phi(phi0, phiV, phiU, S));
		}
	}
}
*/

double AclothoidLine::angle_diff(double theta1, double theta2){
	double pi = UKismetMathLibrary::GetPI();
	double diff = theta1 - theta2; // �������v�Z
	while (diff > pi) diff -= 2 * pi; // �������΂��傫���ꍇ��2�΂����炷
	while (diff < -pi) diff += 2 * pi; // ������-�΂�菬�����ꍇ��2�΂����₷
	return diff; // ���K�����ꂽ������Ԃ�
}

float AclothoidLine::get_clothoid_angle(float radius)
{
	double scale = 1; //�X�P�[�����q
	double L = scale * UKismetMathLibrary::Sqrt(radius);
	float angle = pow(L,2) / (2 * pow(scale,2));
	return angle;
}

double AclothoidLine::fx(int n, float phi, double x,float fov)
{
	float stepS = 1.0f / n;
	TArray<FVector2D> points;
	FPhiSlope pSlope;
	

	pSlope.phiV = UKismetMathLibrary::DegreesToRadians(x);
	pSlope.phiU = phi - pSlope.phiV;

	//�^����ꂽ����p�ƈ�v���邩�v�Z

	complex<float> psiP_Vector;
	for (int i = 0; i < n; ++i) {
		float S = stepS * i;

		complex<float> r;
		phiSimpson_integral(pSlope, S, S + stepS, &r);
		psiP_Vector += r;

		float x = psiP_Vector.real();
		float y = psiP_Vector.imag();
		points.Add(1 * FVector2D(x, y));
	}

	// 2�_�̍��W���烉�W�A�������߂�
	double radian = atan2(points[points.Num() - 1].Y - points[0].Y, points[points.Num() - 1].X - points[0].X);

	//degree = UKismetMathLibrary::RadiansToDegrees(radian)
	double diff = angle_diff(radian, UKismetMathLibrary::DegreesToRadians(fov));

	return UKismetMathLibrary::RadiansToDegrees(diff);

	//double degreeDiff = UKismetMathLibrary::RadiansToDegrees(diff);

	//return degreeDiff;
	//angleOf2DVector(points[0], points[points.Num() - 1])

	// ���W�A������x�����߂�
	//double degree = UKismetMathLibrary::RadiansToDegrees(radian);

	//return degree - fov;
	//return angleOf2DVector(points[0],points[points.Num() - 1]) - fov;
}

float AclothoidLine::calcTurnRadius(float turn_speed, float turnningPerformance,float scale)
{
	
	float angle = FMath::Clamp(turnningPerformance,0.0f,89.0f);
	//���x��kt(�m�b�g)����m/s�ɕύX
	double metorSpeed = speed * 1.852 * 1000 / 3600;
	//�o���N�p�����W�A���ɕϊ�
	double radian_bankAngle = angle* UKismetMathLibrary::GetPI()/180;

	//���񔼌a����(R=V^2/(g(9.8)*tan��))
	double result = pow(metorSpeed, 2) / (9.8 * tan(radian_bankAngle));

	//�l��scale�l�Ŕ�����
	return result / scale;
}

float AclothoidLine::myNewton(float n, float _phi, float fov){
	double val = 999;
	int k = 0;
	int countMax = 100;
	double rangeA = -1000;
	double rangeB = 1000;
	double eps = 0.1;
	double mid = 0;
	double value = 0;
	if(rangeB - rangeA == 0){
		return value;
	}else{
		mid = (rangeB - rangeA) / 2 + rangeA;
	}

	while (k < countMax){
		if(abs(fx(n,_phi,rangeA,fov)) > abs(fx(n,_phi,rangeB,fov))){
			if(abs(fx(n,_phi,rangeB,fov)) < eps){
				value = rangeB;
				break;
			}
			rangeA = mid;
		}else{
			if(abs(fx(n, _phi, rangeA, fov)) < eps){
				value = rangeA;
				break;
			}
			rangeB = mid;
		}
			
		mid = (rangeB - rangeA) / 2 + rangeA;
		k += 1;
	}
		

	if (k >= countMax){
		UE_LOG(LogTemp, Error, TEXT("Failed to converge!!"));
	}

	UE_LOG(LogTemp, Log, TEXT("(result : %f,n : %f,phi : %f,fov : %f),count %d"),value, n,UKismetMathLibrary::RadiansToDegrees(_phi), fov, k);

	return value;
}

float AclothoidLine::getPhi_newtonMethod(float n,float _phi,float fov){
	float error = 0.01;
	float delta = 0.001;
	float x1 = 0;
	int k = 0;
	int countMax = 100;

	while (true) {
		float x2 = x1 + delta;
		float y1 = fx(n,_phi,x1,fov);
		float y2 = fx(n, _phi,x2,fov);
		float diff_x = (y2 - y1) / (x2 - x1);
		float next_x = x1 - y1 / diff_x;

		if (std::fabs(y1 - 0) < error || k >= countMax) {
			break;
		}
		x1 = next_x;
		k += 1;
	}

	if (k >= countMax) {
		UE_LOG(LogTemp, Error, TEXT("Failed to converge!!"));
	}
	UE_LOG(LogTemp, Log, TEXT("(result : %f,n : %f,phi : %f,fov : %f),count %d"), x1, n, UKismetMathLibrary::RadiansToDegrees(_phi), fov, k);

	return x1;
}

float AclothoidLine::falsePositionMethod(float n,float _phi,float fov){
	float _phi_x_Value = 0;
	//param
	double x_start1 = -500;
	double x_start2 = 500;
	double err = 100.0;
	double a = x_start1;
	double b = x_start2;
	//float x = 0;
	int countMax = 10;
	int k = 0;



	while (err > 0.00000001 && k < countMax) {
		_phi_x_Value = (a * fx(n,_phi, b, fov) - b * fx(n, _phi, a, fov)) / (fx(n, _phi, b, fov) - fx(n, _phi, a, fov));
		double fc = fx(n, _phi, _phi_x_Value, fov);
		err = fc * fc;
		UE_LOG(LogTemp, Log, TEXT("falsePositionMethod Log (err : %f)"), err);
		if (fc < 0) {
			b = _phi_x_Value;
		}
		else {
			a = _phi_x_Value;
		}
		k++;
	}

	if (k >= countMax) {
		UE_LOG(LogTemp, Error, TEXT("Failed to converge!!"));
	}
	UE_LOG(LogTemp, Log, TEXT("(result : %f,n : %f,phi : %f,fov : %f),count %d"), _phi_x_Value, n,UKismetMathLibrary::RadiansToDegrees(_phi), fov, k);


return _phi_x_Value;
}

FVector AclothoidLine::calcCurveHandle(FVector p1, FVector p2, float radius){
	float dis = FVector::Distance(p1,p2);
	float cosd = UKismetMathLibrary::DegCos(radius);
	FRotator rot(0, UKismetMathLibrary::FindLookAtRotation(p1, p2).Yaw + radius,0);

	FVector fv = UKismetMathLibrary::GetForwardVector(rot);
	return p1 + (fv * (dis/(cosd*2.0f)));
}

float AclothoidLine::getThreePointAngle(FVector p1, FVector p2, FVector p3)
{
	FVector vec1 = p1 - p2;
	FVector vec2 = p2 - p3;
	float dotNum = UKismetMathLibrary::Dot_VectorVector(vec1,vec2);
	FVector crossNum = UKismetMathLibrary::Cross_VectorVector(vec1,vec2);

	return UKismetMathLibrary::DegAtan2(crossNum.Size(), dotNum) 
	* UKismetMathLibrary::SignOfFloat(crossNum.Z);
}

float AclothoidLine::getMiddleAngle(float angle1, float angle2)
{
	float angle1R =  UKismetMathLibrary::DegreesToRadians(angle1);
	float angle2R = UKismetMathLibrary::DegreesToRadians(angle2);

	float x = (UKismetMathLibrary::Cos(angle1R) + UKismetMathLibrary::Cos(angle2R)) / 2;
	float y = (UKismetMathLibrary::Sin(angle1R) + UKismetMathLibrary::Sin(angle2R)) / 2;

	return UKismetMathLibrary::DegAtan2(y,x);
}

float AclothoidLine::convert_Angle_to_controllAngle(float angle)
{
	if(UKismetMathLibrary::InRange_FloatFloat(angle,-180,-90,true,true)){
		return UKismetMathLibrary::Abs(angle)-180;
	}
	if (UKismetMathLibrary::InRange_FloatFloat(angle, -90, 0, false, false)) {
		return UKismetMathLibrary::Abs(angle) - 180;
	}
	if (UKismetMathLibrary::InRange_FloatFloat(angle,0,180, true, true)) {
		return 180 - angle;
	}
	return 0.0f;
}

FVector AclothoidLine::getAngleLocation(float angle, float radius, FVector centerLocation)
{
	return centerLocation + FVector(
	radius * UKismetMathLibrary::DegCos(angle),
	radius * UKismetMathLibrary::DegSin(angle),
	0.0f
	);
}

FVector AclothoidLine::getAngleLocationFromThreepoint(float thetaMax,FVector p1, FVector p2, FVector p3,float radius)
{
	
	float r1 = UKismetMathLibrary::FindLookAtRotation(p2,p1).Yaw;
	float r2 = UKismetMathLibrary::FindLookAtRotation(p2, p3).Yaw;
	FRotator angle_Middle = FRotator(0, getMiddleAngle(r1, r2),0);

	FRotator composeRot = UKismetMathLibrary::ComposeRotators(angle_Middle,FRotator(0,180,0));
	float angle_threePoint = getThreePointAngle(p1, p2, p3);
	FRotator deltaRot = FRotator(0, UKismetMathLibrary::SignOfFloat(angle_threePoint) * turn_rate,0);

	float angle_circle = UKismetMathLibrary::NormalizedDeltaRotator(composeRot,deltaRot).Yaw;
	FVector center = p2 + UKismetMathLibrary::GetForwardVector(angle_Middle) * radius;

	return getAngleLocation(angle_circle,radius,center);
}

TArray<FVector2D> AclothoidLine::calcClothoidCurve(int n,float phi1, float phi0,float straightDis,float fov){
	
    float stepS = 1.0f / 50;
	float iota = straightDis;
    TArray<FVector2D> psiPoints;
    TArray<FVector2D> psiCalcuPoints;

	TArray<FVector2D> cPoints;

    FSlope slope;
    slope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
    slope.phiV = 0;
    slope.phiU = 0;

	FPhiSlope phiSlope;

	
	/*
    //���݌����@�Ŏg���֐�
	auto f = [&n, &phi1, &phi0](double x) {
        float stepS = 1.0f / n;
        TArray<FVector2D> points;
        FPhiSlope pSlope;

		pSlope.phiV = UKismetMathLibrary::DegreesToRadians(x);
		pSlope.phiU = UKismetMathLibrary::DegreesToRadians(phi1 - phi0) - pSlope.phiV;

        //�^����ꂽ����p�ƈ�v���邩�v�Z
        complex<float> psiP_Vector;
        for (int i = 0; i < n; ++i) {
            float S = stepS * i;

            complex<float> r;
            phiSimpson_integral(pSlope, S, S + stepS, &r);
            psiP_Vector += r;

            float x = psiP_Vector.real();
            float y = psiP_Vector.imag();
            points.Add(1 * FVector2D(x, y));
        }

        // 2�_�̍��W���烉�W�A�������߂�
        double radian = atan2(points[points.size() - 1].y - points[0].y, points[points.size() - 1].x - points[0].x);

        // ���W�A������x�����߂�
        double degree = radian * 180 / pi;

        return degree;
    };
	*/

	/*
    //���݌����@
    while (err > 0.00000001 || k > countMax) {
        x = (a * f(b) - b * f(a)) / (f(b) - f(a));
        double fc = f(x);
        err = fc * fc;
        if (fc < 0) {
            b = x;
        }
        else {
            a = x;
        }
        k++;
    }
    if (k > countMax) {
		UE_LOG(LogTemp, Error, TEXT("�������s"));
    }
	*/

	/*
	// Newton-Raphson method
	double initialX = a; // initial guess
	double err = 1.0;
	int k = 0;
	while (err > 0.00000001 && k < countMax) {
		double fx = fx(n, _tempPhi, x, fov);
		double fpx = fpx(n, _tempPhi, x, fov); // derivative of fx
		double x_new = x - fx / fpx;
		err = abs(x_new - x);
		x = x_new;
		k++;
	}
	if (k >= countMax) {
		UE_LOG(LogTemp, Error, TEXT("Failed to converge (phi1 : %f , phi0 : %f )"), phi1, phi0);
	}
	*/

	float _tempPhi = UKismetMathLibrary::DegreesToRadians(phi1 - phi0);
	UE_LOG(LogTemp, Error, TEXT("phi1 is %f,phi0 is %f"), phi1,phi0);

	//���݌����@�Ōv�Z
	
	//-1.1491665778430064
	phiSlope.phiV = UKismetMathLibrary::DegreesToRadians(getPhi_newtonMethod(n, _tempPhi,fov));
    phiSlope.phiU = UKismetMathLibrary::DegreesToRadians(phi1 - phi0) - phiSlope.phiV;
	phiSlope.phiU = _tempPhi - phiSlope.phiV;

    complex<float> psiP_Vector;
    for (int i = 0; i < 50; ++i) {
        float S = stepS * i;

        complex<float> r;
        phiSimpson_integral(phiSlope, S, S + stepS, &r);
        psiP_Vector += r;

        float l_x = psiP_Vector.real();
        float l_y = psiP_Vector.imag();
        psiPoints.Add(1 * FVector2D(l_x, l_y));
    }
	
    double lamda = UKismetMathLibrary::Distance2D(psiPoints[psiPoints.Num() - 1],psiPoints[0]);

    float h = iota / lamda;

    FSlope cSlope;
    cSlope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
    cSlope.phiV = phiSlope.phiV;
    cSlope.phiU = phiSlope.phiU;

	stepS = 1.0f / n;

    //�N���\�C�h���
    complex<float> cP_Vector;
    for (int i = 0; i < n; ++i) {
        float S = stepS * i;

        complex<float> r;
        simpson_integral(cSlope, S, S + stepS, &r);
        cP_Vector += r;

        float l_x = cP_Vector.real();
        float l_y = cP_Vector.imag();
        cPoints.Add(h * FVector2D(l_x, l_y));
    }
	return cPoints;
}

float AclothoidLine::angleOf2DVector(FVector2D p1, FVector2D p2)
{
	static const double pi = 3.14159265358979323846;
	// 2�_�̍��W���烉�W�A�������߂�
	//double radian = (FGenericPlatformMath::Atan2(p2.Y - p1.Y, p2.X - p1.X));
	double radian = (atan2(p2.Y - p1.Y, p2.X - p1.X));

	// ���W�A������x�����߂�
	return  radian * 180 / pi;
}


TArray<FVector> AclothoidLine::calcClothoidSpline(UPARAM(ref) TArray<FClothoidPath>& pathDatas, float radius = 0){
	TArray<FVector>results;

	if (pathDatas.Num() <= 1) {
		UE_LOG(LogTemp, Error, TEXT("TArray.Num() is %d"), pathDatas.Num());
		return results;
	}

	int splineCount = pathDatas.Num() - 1;

	float s = 0.5f;
	phiValues.Empty();

	//float _hValue;
	
	//float _phiVValue;
	//float _phiUValue;

	

	int current = 0;
	int target = 1;
	int next = 2;
	FVector currentLoc = pathDatas[current].p_controll_Loc;
	FVector2D currentLocXY = FVector2D(currentLoc.X,currentLoc.Y);
	//FVector2D currentLocYZ = FVector2D(currentLoc.Y, currentLoc.Z);
	FVector targetLoc;
	FVector2D targetLocXY;
	
	results.Add(currentLoc);

	for(; current < splineCount;){
		
		//����_�Ԃ̏����p�x�ݒ�
		float _phi0Value = pathDatas[current].phi0;
		float _phi1Value = pathDatas[current].phi1;
		
		//���̐���_�̍��W�ݒ�
		targetLoc = pathDatas[target].p_controll_Loc;
		targetLocXY = FVector2D(targetLoc.X, targetLoc.Y);
		
		//�����擾
		float _hValue = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);

		//�N���\�C�h�Ȑ��̃X�e�b�v�ϐ�(1�Ő��K��)
		int num = _hValue * ((float)accuracy / 100.0);
		float stepS = 1.0f / num;

		//����p�x
		float turnnningAngle = 0;
		
		//�ȗ��ɂ��N���\�C�h���䂷�邽�߂̐���_���W
		FVector controll_p_Location;
		FVector2D controllP;
		//���񔼌a
		float r = calcTurnRadius(speed, turningPerformance, 250);

		//�~�ʏ�̐ڐ��p�x
		float _tangetAngleOnCircle;

		//���̃p�X������ꍇ
		if(pathDatas.IsValidIndex(next)){

			//���񔼌a���p�X�Ԃ̋����̔������������ꍇ�A�p�X�Ԃ̋����̔�������񔼌a�ɂ���
			if((_hValue/2)<=radius){
				radius = _hValue / 2;
			}
			float r1 = UKismetMathLibrary::FindLookAtRotation(targetLoc, currentLoc).Yaw;
			float r2 = UKismetMathLibrary::FindLookAtRotation(targetLoc, pathDatas[next].p_controll_Loc).Yaw;
			FRotator angle_Middle = FRotator(0, getMiddleAngle(r1, r2), 0);

			FRotator composeRot = UKismetMathLibrary::ComposeRotators(angle_Middle, FRotator(0, 180, 0));
			float angle_threePoint = getThreePointAngle(currentLoc, targetLoc, pathDatas[next].p_controll_Loc);
			float signVal = UKismetMathLibrary::SignOfFloat(angle_threePoint);
			FRotator deltaRot = FRotator(0, signVal * turn_rate, 0);

			float angle_circle = UKismetMathLibrary::NormalizedDeltaRotator(composeRot, deltaRot).Yaw;
			FVector center = targetLoc + UKismetMathLibrary::GetForwardVector(angle_Middle) * radius;

			//���񔼌a�̉~�ʂ̎n�܂�𐧌�_�ɐݒ肷��
			controll_p_Location = getAngleLocation(angle_circle, radius, center);
			//�~�ʏ�̐ڐ��̊p�x���擾����
			_tangetAngleOnCircle = UKismetMathLibrary::ComposeRotators(UKismetMathLibrary::FindLookAtRotation(center, controll_p_Location), FRotator(0, signVal * 90, 0)).Yaw;

			//controll_p_Location = getAngleLocationFromThreepoint(turn_rate,currentLoc,targetLoc,pathDatas[next].p_controll_Loc,radius);
		}else{//���̃p�X���Ȃ��ꍇ
			//�Ώۂ̃p�X���W�ɐ���_�ɐݒ肷��
			controll_p_Location = targetLoc;
			
			//�~�ʂ������̂Ō��݂̃p�X���W����Ώۂ̃p�X���W�ւ̊p�x�����̂܂ܐ���_�ɐݒ肷��
			FVector vec = targetLoc - currentLoc;
			_tangetAngleOnCircle = convert_Angle_to_controllAngle(UKismetMathLibrary::DegAtan2(vec.Y,vec.X));
		}

		//���ݒn���玟�̓_�܂ł̊Ԃɐݒ肳�ꂽ�ȗ����珉���ȗ�(phi0)�Ɠ��B�ȗ�(phi1)��ݒ�
		//FVector top = calcCurveHandle(currentLoc,pathDatas[target],radius);

		/*
		if(_phi1Value - _phi0Value == 0){
			_phi0Value = UKismetMathLibrary::FindLookAtRotation(FVector(currentLocXY.X, currentLocXY.Y, 0), /FVector(top.X, top.Y, 0)).Yaw;
			_phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(top.X, top.Y, 0), FVecto(targetLocXY.X, //targetLocXY.Y, 0)).Yaw;
		}
		*/
		//float _phi0Value = UKismetMathLibrary::FindLookAtRotation(FVector(currentLocXY.X, currentLocXY.Y, 0), /FVector(top.X, top.Y, 0)).Yaw;

		//float _phi0Value = test_phiZeroValue;

		//float _fov = _phi1Value - _phi0Value;

		//����p�v�Z
		float _fov = angleOf2DVector(currentLocXY, targetLocXY) - _phi0Value;

		//float _phi1Value = test_phiOneValue;
		//float _phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(top.X, top.Y, 0), FVector/(targetLocXY.X, targetLocXY.Y, 0)).Yaw;
		
		/*
		//�Ώۂ̎��̃p�X�����݂��Ȃ���
		if (!pathDatas.IsValidIndex(next)) {
			//�p�x��ύX�����ɂ��̂܂܂̊p�x��ݒ肷��
			_phi1Value = _phi0Value;
			
		}
		else {
			//�Ώۂ̃p�X����Ώۂ̎��̃p�X�܂ł̊p�x��ݒ肷��
			FVector2D nextLoc(pathDatas[next].X, pathDatas[next].Y);
			_phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(targetLocXY.X, targetLocXY.Y, 0),FVector//(nextLoc.X, nextLoc.Y, 0)).Yaw;

		}
		*/

		//�N���\�C�h�Ȑ��𐶐�
		TArray<FVector2D>curve = calcClothoidCurve(num, _phi1Value, _phi0Value, _hValue,_fov);

		//���̍����ݒ�
		float currentZ = currentLoc.Z;
		float zDiff = targetLoc.Z - currentLoc.Z;
		for (int k = 0; k < num; k++) {
			results.Add(FVector(curve[k].X, curve[k].Y, 0) + FVector(currentLocXY.X, currentLocXY.Y, currentZ + zDiff * (stepS * k)));
		}

		//���ʂ̍ŏI���W�����̃N���\�C�h�Ȑ��v�Z�̃X�^�[�g�n�_�ɐݒ�
		currentLoc = results[results.Num() - 1];
		currentLocXY = FVector2D(currentLoc.X, currentLoc.Y);

		curve.Empty();
		current++;
		target++;
		next++;
	}

	return results;
}

/*
TArray<FVector> AclothoidLine::CalcClothoidSplineResultsSecond(UPARAM(ref) TArray<FVector>& pathDatas){
	TArray<FVector>results;

	if(pathDatas.Num()<=1){
		UE_LOG(LogTemp,Error, TEXT("TArray.Num() is %d"),pathDatas.Num());
		return results;
	}

	if(accuracy==0){
		UE_LOG(LogTemp, Error, TEXT("accuracy is %d!!"),accuracy);
		return results;
	}

	int splineCount = pathDatas.Num()-1;

	float s = 0.5f;

	//float _hValue;
	//float _phi0Value;
	//float _phiVValue;
	//float _phiUValue;

	float stepS = 1.0f / accuracy;

	int current =0;
	int target = 1;
	int next = 2;
	FVector2D currentLoc = FVector2D(pathDatas[current].X, pathDatas[current].Y);
	FVector2D targetLoc;
	results.Add(FVector(currentLoc.X,currentLoc.Y,pathDatas[current].Z));

	for(;current<splineCount;){
		//�����擾
		targetLoc = FVector2D(pathDatas[target].X, pathDatas[target].Y);
		float _hValue = UKismetMathLibrary::Distance2D(targetLoc, currentLoc);

		//���ݒn����Ώۂ܂ł̊p�x�������p�x�Ƃ��Đݒ肷��
		float _phi0Value = UKismetMathLibrary::FindLookAtRotation(FVector(currentLoc.X, currentLoc.Y,0),FVector(targetLoc.X, targetLoc.Y, 0)).Yaw;

		float _fov = angleOf2DVector(currentLoc, targetLoc) - _phi0Value;
		
		float _phi1Value = 0;
		//�Ώۂ̎��̃p�X�����݂��Ȃ�
		if(!pathDatas.IsValidIndex(next)){
			//�p�x��ύX�����ɂ��̂܂܂̊p�x��ݒ肷��
			 _phi1Value = _phi0Value;
		//�Ώۂ̎��̃p�X�����݂���
		}else{
			//�Ώۂ̃p�X����Ώۂ̎��̃p�X�܂ł̊p�x��ݒ肷��
			FVector2D nextLoc(pathDatas[next].X, pathDatas[next].Y);
			_phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(targetLoc.X, targetLoc.Y, 0), FVector(nextLoc.X, nextLoc.Y, 0)).Yaw;
			 
		}

		TArray<FVector2D>curve = calcClothoidCurve(accuracy, _phi1Value, _phi0Value, _hValue, _fov);
		
		//���̍����ݒ�
		float currentZ = pathDatas[current].Z;
		float zDiff = pathDatas[target].Z-pathDatas[current].Z;

		for(int k =0;k<accuracy;k++){
			results.Add(FVector(curve[k].X,curve[k].Y,0) + FVector(currentLoc.X,currentLoc.Y,currentZ + zDiff * (stepS*k)));
		}

		FVector lastLoc(results[results.Num() - 1]);
		currentLoc = FVector2D(lastLoc.X,lastLoc.Y);
		
		curve.Empty();
		current++;
		target++;
		next++;
	}

	return results;
}
*/

