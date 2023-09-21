
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
	//定義
	TFunctionRef<float(const int32)> TestLambda = [&Value1](const int32 Value2)
	{
		return static_cast<float>(Value1 + Value2);
	};
	//呼び出し
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
	double diff = theta1 - theta2; // 差分を計算
	while (diff > pi) diff -= 2 * pi; // 差分がπより大きい場合は2πずつ減らす
	while (diff < -pi) diff += 2 * pi; // 差分が-πより小さい場合は2πずつ増やす
	return diff; // 正規化された差分を返す
}

float AclothoidLine::get_clothoid_angle(float radius)
{
	double scale = 1; //スケール因子
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

	//与えられた視野角と一致するか計算

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

	// 2点の座標からラジアンを求める
	double radian = atan2(points[points.Num() - 1].Y - points[0].Y, points[points.Num() - 1].X - points[0].X);

	//degree = UKismetMathLibrary::RadiansToDegrees(radian)
	double diff = angle_diff(radian, UKismetMathLibrary::DegreesToRadians(fov));

	return UKismetMathLibrary::RadiansToDegrees(diff);

	//double degreeDiff = UKismetMathLibrary::RadiansToDegrees(diff);

	//return degreeDiff;
	//angleOf2DVector(points[0], points[points.Num() - 1])

	// ラジアンから度を求める
	//double degree = UKismetMathLibrary::RadiansToDegrees(radian);

	//return degree - fov;
	//return angleOf2DVector(points[0],points[points.Num() - 1]) - fov;
}

float AclothoidLine::calcTurnRadius(float turn_speed, float turnningPerformance,float scale)
{
	
	float angle = FMath::Clamp(turnningPerformance,0.0f,89.0f);
	//速度をkt(ノット)からm/sに変更
	double metorSpeed = speed * 1.852 * 1000 / 3600;
	//バンク角をラジアンに変換
	double radian_bankAngle = angle* UKismetMathLibrary::GetPI()/180;

	//旋回半径公式(R=V^2/(g(9.8)*tanΘ))
	double result = pow(metorSpeed, 2) / (9.8 * tan(radian_bankAngle));

	//値をscale値で微調整
	return result / scale;
}

/*
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
*/

float AclothoidLine::getPhi_newtonMethod(float n,float _phi,float fov){
	float error = 0.01;
	float delta = 0.001;
	float x1 = 0;
	int k = 0;
	int countMax = 30;

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

/*
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
*/

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

FVector AclothoidLine::calcStartLocationOnArc(FVector current,float arc_radius,FVector center,float sign)
{
	
	FVector vec = center - current;
	float rad = atan2(vec.Y,vec.X);
	float dis = vec.Size();
	float _theta = UKismetMathLibrary::Asin(arc_radius /dis)*sign + rad;
	float _x = arc_radius * cos(_theta);
	float _y = arc_radius * sin(_theta);

	return FVector(_x + current.X,_y + current.Y,center.Z);
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


float AclothoidLine::calculateArcLength(float radius, float startAngle, float endAngle)
{
	return radius* (UKismetMathLibrary::DegreesToRadians(endAngle) - UKismetMathLibrary::DegreesToRadians(startAngle));
}

TArray<FVector> AclothoidLine::calcCircleLocations(float startAngle, float endAngle,float radius,FVector curve_start,FVector curve_end)
{
	TArray<FVector>results;

	float zDiff = curve_end.Z - curve_start.Z;
	
	float angleLength = angle_diff(UKismetMathLibrary::DegreesToRadians(endAngle), UKismetMathLibrary::DegreesToRadians(startAngle));
	angleLength = UKismetMathLibrary::RadiansToDegrees(angleLength);
	int num = UKismetMathLibrary::Abs(angleLength)* ((float)arc_accuracy / 100.0);
	if(num < 1){
		results.Add(curve_start);
		return results;
	}

	float step = 1.0f / num;

	FVector center = FVector::ZeroVector;
	FVector startAngleLoc = getAngleLocation(startAngle, radius, center);
	results.SetNum(num);
	for(int i = 0;i<num;i++){
		float s = i * step;
		float arc_point_angle = UKismetMathLibrary::ComposeRotators(FRotator(0, startAngle, 0), FRotator(0, angleLength * s, 0)).Yaw;
		FVector arcPoint = getAngleLocation(arc_point_angle,radius,center);
		//results[i] = cur + FVector2D(cirLoc);
		results[i] = FVector(curve_start.X + arcPoint.X - startAngleLoc.X, curve_start.Y + arcPoint.Y - startAngleLoc.Y, curve_start.Z + (zDiff * s));
	}

	return results;
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

TArray<FVector> AclothoidLine::calcClothoidCurve(int n,float phi1, float phi0,float straightDis,float fov,FVector controllLoc,FVector curveStartLoc){
	
	int num_CalcClothoid = 50;
    float stepS = 1.0f / num_CalcClothoid;
	float iota = straightDis;
    TArray<FVector2D> psiPoints;
    TArray<FVector2D> psiCalcuPoints;

	TArray<FVector> results;

    FSlope slope;
    slope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
    slope.phiV = 0;
    slope.phiU = 0;

	FPhiSlope phiSlope;

	
	/*
    //挟み撃ち法で使う関数
	auto f = [&n, &phi1, &phi0](double x) {
        float stepS = 1.0f / n;
        TArray<FVector2D> points;
        FPhiSlope pSlope;

		pSlope.phiV = UKismetMathLibrary::DegreesToRadians(x);
		pSlope.phiU = UKismetMathLibrary::DegreesToRadians(phi1 - phi0) - pSlope.phiV;

        //与えられた視野角と一致するか計算
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

        // 2点の座標からラジアンを求める
        double radian = atan2(points[points.size() - 1].y - points[0].y, points[points.size() - 1].x - points[0].x);

        // ラジアンから度を求める
        double degree = radian * 180 / pi;

        return degree;
    };
	*/

	/*
    //挟み撃ち法
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
		UE_LOG(LogTemp, Error, TEXT("収束失敗"));
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

	//挟み撃ち法で計算
	
	//-1.1491665778430064
	phiSlope.phiV = UKismetMathLibrary::DegreesToRadians(getPhi_newtonMethod(n, _tempPhi,fov));
    phiSlope.phiU = UKismetMathLibrary::DegreesToRadians(phi1 - phi0) - phiSlope.phiV;
	phiSlope.phiU = _tempPhi - phiSlope.phiV;

	psiPoints.SetNum(num_CalcClothoid);
    complex<float> psiP_Vector;
    for (int i = 0; i < num_CalcClothoid; ++i) {
        float S = stepS * i;

        complex<float> r;
        phiSimpson_integral(phiSlope, S, S + stepS, &r);
        psiP_Vector += r;

        float l_x = psiP_Vector.real();
        float l_y = psiP_Vector.imag();
		
		psiPoints[i] = (1 * FVector2D(l_x, l_y));
        //psiPoints.Add(1 * FVector2D(l_x, l_y));
    }
	
    double lamda = UKismetMathLibrary::Distance2D(psiPoints[psiPoints.Num() - 1],psiPoints[0]);

    float h = iota / lamda;

    FSlope cSlope;
    cSlope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
    cSlope.phiV = phiSlope.phiV;
    cSlope.phiU = phiSlope.phiU;

	stepS = 1.0f / n;

	float zDiff = curveStartLoc.Z - controllLoc.Z;

    //クロソイド補間
    complex<float> cP_Vector;
	results.SetNum(n);
    for (int i = 0; i < n; ++i) {
        float S = stepS * i;

        complex<float> r;
        simpson_integral(cSlope, S, S + stepS, &r);
        cP_Vector += r;

        float l_x = cP_Vector.real();
        float l_y = cP_Vector.imag();
        //cPoints.Add(h * FVector2D(l_x, l_y));
		FVector2D clothoidCurve = (h * FVector2D(l_x, l_y));
		results[i] = FVector(clothoidCurve.X + controllLoc.X,clothoidCurve.Y + controllLoc.Y,controllLoc.Z + zDiff * S);
    }
	return results;
}

float AclothoidLine::angleOf2DVector(FVector2D p1, FVector2D p2)
{
	static const double pi = 3.14159265358979323846;
	// 2点の座標からラジアンを求める
	//double radian = (FGenericPlatformMath::Atan2(p2.Y - p1.Y, p2.X - p1.X));
	double radian = (atan2(p2.Y - p1.Y, p2.X - p1.X));

	// ラジアンから度を求める
	return  radian * 180 / pi;
}

float AclothoidLine::calc_TurnRate(FVector target, FVector p1, FVector p2)
{
	float angle1 = UKismetMathLibrary::FindLookAtRotation(target, p1).Yaw;
	float angle2 = UKismetMathLibrary::FindLookAtRotation(target, p2).Yaw;
	float _angleDiff = angle_diff(UKismetMathLibrary::DegreesToRadians(angle1), UKismetMathLibrary::DegreesToRadians(angle2));
	return UKismetMathLibrary::RadiansToDegrees(abs(_angleDiff));
}

TArray<FVector> AclothoidLine::calcClothoidSpline(UPARAM(ref) TArray<FClothoidPath>& pathDatas){
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
		
		//制御点間の初期角度設定
		//float _phi0Value = pathDatas[current].phi0;
		//float _phi1Value = pathDatas[current].phi1;
		float turn_rate_start = 0;
		float turn_rate_end = 0;
		
		//次の制御点の座標設定
		targetLoc = pathDatas[target].p_controll_Loc;
		targetLocXY = FVector2D(targetLoc.X, targetLoc.Y);

		//旋回角度
		float turnnningAngle = 0;
		
		//クロソイドにつなげる円弧座標
		FVector arc_start;
		FVector2D arc_start_XY;
		FVector arc_end;

		//旋回半径
		float radius = calcTurnRadius(speed, turningPerformance, scale_calcTurnRadius);
		//始点角度
		float startAngle = 0;
		//終点角度
		float endAngle = 0;

		//円弧上の接線角度
		float _tangentAngleOnCircle = 0;

		//float _phi0Value = pathDatas[current].phi0;
		//float _phi1Value = pathDatas[current].phi1;

		//次のパスがある場合
		if(pathDatas.IsValidIndex(next)){
			
			float dis_To_p_controll = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);
			//旋回半径がパス間の距離の半分をこえた場合、パス間の距離の半分を旋回半径にする
			if((dis_To_p_controll / 2)<=radius){
				radius = dis_To_p_controll / 2;
			}
			float r1 = UKismetMathLibrary::FindLookAtRotation(targetLoc, currentLoc).Yaw;
			float r2 = UKismetMathLibrary::FindLookAtRotation(targetLoc, pathDatas[next].p_controll_Loc).Yaw;
			FRotator angle_Middle = FRotator(0, getMiddleAngle(r1, r2), 0);

			FRotator arc_Rot = UKismetMathLibrary::ComposeRotators(angle_Middle, FRotator(0, 180, 0));
			float angle_threePoint = getThreePointAngle(currentLoc, targetLoc, pathDatas[next].p_controll_Loc);
			float signVal = UKismetMathLibrary::SignOfFloat(angle_threePoint);
			//turn_rateの大きさで始点角度が決まる。（turn_rate : 0<=X<=90) 
			//ここのturn_rateは始点角度の変化量
			//実装予定(turn_rateを始点角度変化量に変更、終点角度変化量変数を作成する必要アリ)
			
			//angle_Middle.Yaw - r1
			
			

			//float angle_circle = UKismetMathLibrary::NormalizedDeltaRotator(composeRot, deltaRot).Yaw;
			
			turn_rate_start = calc_TurnRate(currentLoc,targetLoc,pathDatas[next].p_controll_Loc);
			turn_rate_end = calc_TurnRate(pathDatas[next].p_controll_Loc, targetLoc, currentLoc);

			FRotator startRot = FRotator(0, signVal * turn_rate_start, 0);
			//終点角度計算は未実装
			//FRotator endRot = FRotator(0, -1 * signVal * turn_rate, 0);
			FRotator endRot = FRotator(0, -1 * signVal * turn_rate_end, 0);

			startAngle = UKismetMathLibrary::NormalizedDeltaRotator(arc_Rot, startRot).Yaw;
			endAngle = UKismetMathLibrary::NormalizedDeltaRotator(arc_Rot, endRot).Yaw;
				
			FVector center = targetLoc + UKismetMathLibrary::GetForwardVector(angle_Middle) * radius;

			//旋回半径の円弧の始まり
			arc_start = getAngleLocation(startAngle, radius, center);

			FVector arcstart1 = calcStartLocationOnArc(currentLoc, radius, center, signVal); 
			test_arcStartLoc = currentLoc;
			float startAngle1 = UKismetMathLibrary::FindLookAtRotation(center, arcstart1).Yaw;

			arc_end = getAngleLocation(endAngle, radius, center);

			//高さ計算
			float arcLen = calculateArcLength(radius, startAngle, endAngle);
			float curveLen = dis_To_p_controll + arcLen;
			float zDiff = targetLoc.Z - currentLoc.Z;
			arc_start.Z = ((dis_To_p_controll / curveLen) * zDiff) + currentLoc.Z;
			arc_end.Z = targetLoc.Z;

			//円弧上の接線の角度を取得する
			_tangentAngleOnCircle = UKismetMathLibrary::ComposeRotators(UKismetMathLibrary::FindLookAtRotation(center, arc_start), FRotator(0, signVal * 90, 0)).Yaw;

			//controll_p_Location = getAngleLocationFromThreepoint(turn_rate,currentLoc,targetLoc,pathDatas[next].p_controll_Loc,radius);
		}else{//次のパスがない場合
			//対象のパス座標に制御点に設定する
			arc_start = targetLoc;
			arc_end = targetLoc;
			
			//円弧が無いので現在のパス座標から対象のパス座標への角度をそのまま制御点に設定する
			FVector vec = targetLoc - currentLoc;
			_tangentAngleOnCircle = UKismetMathLibrary::DegAtan2(vec.Y, vec.X);
			//始点角度、終点角度はそのまま0で設定
		}

		FVector vec_tangent = arc_start - currentLoc;

		arc_start_XY = FVector2D(arc_start.X, arc_start.Y);

		//距離取得
		//float _hValue = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);
		float _hValue = UKismetMathLibrary::Distance2D(arc_start_XY, currentLocXY);
		int num = _hValue * ((float)accuracy / 100.0);
		//int num = accuracy;
		float stepS = 1.0f / num;

		//_hValue = 

		//このphi0は、前の制御点のphi1を考慮せず、計算している
		//実装予定
		//float _phi0Value = convert_Angle_to_controllAngle(UKismetMathLibrary::DegAtan2(vec_tangent.Y, vec_tangent.X));
		float _phi0Value = UKismetMathLibrary::DegAtan2(vec_tangent.Y, vec_tangent.X);

		//float _phi1Value = convert_Angle_to_controllAngle(_tangentAngleOnCircle);
		float _phi1Value = _tangentAngleOnCircle;

		//現在地から次の点までの間に設定された曲率から初期曲率(phi0)と到達曲率(phi1)を設定
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
		//視野角計算
		float _fov = angleOf2DVector(currentLocXY, arc_start_XY) - _phi0Value;
		//float _fov = angleOf2DVector(currentLocXY, targetLocXY) - _phi0Value;

		//float _phi1Value = test_phiOneValue;
		//float _phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(top.X, top.Y, 0), FVector/(targetLocXY.X, targetLocXY.Y, 0)).Yaw;
		
		/*
		//対象の次のパスが存在しないか
		if (!pathDatas.IsValidIndex(next)) {
			//角度を変更せずにそのままの角度を設定する
			_phi1Value = _phi0Value;
			
		}
		else {
			//対象のパスから対象の次のパスまでの角度を設定する
			FVector2D nextLoc(pathDatas[next].X, pathDatas[next].Y);
			_phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(targetLocXY.X, targetLocXY.Y, 0),FVector//(nextLoc.X, nextLoc.Y, 0)).Yaw;

		}
		*/
		//仮の高さ設定
		//float currentZ = currentLoc.Z;
		//float zDiff = targetLoc.Z - currentLoc.Z;

		//クロソイド曲線を生成
		TArray<FVector>curve = calcClothoidCurve(num, _phi1Value, _phi0Value, _hValue,_fov,currentLoc, arc_start);

		//円弧の座標取得
		TArray<FVector>tangent_circles = calcCircleLocations(startAngle,endAngle,radius,curve.Last(),arc_end);
		
		//高さ計算未実装
		/*
		for (int k = 0; k < num; k++) {
			results.Add(FVector(curve[k].X, curve[k].Y, 0) + FVector(currentLocXY.X, currentLocXY.Y, currentZ + zDiff * (stepS * k)));
		}
		*/
		results+=curve;
		results+=tangent_circles;

		//結果の最終座標を次のクロソイド曲線計算のスタート地点に設定
		currentLoc = results.Last();
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
		//距離取得
		targetLoc = FVector2D(pathDatas[target].X, pathDatas[target].Y);
		float _hValue = UKismetMathLibrary::Distance2D(targetLoc, currentLoc);

		//現在地から対象までの角度を初期角度として設定する
		float _phi0Value = UKismetMathLibrary::FindLookAtRotation(FVector(currentLoc.X, currentLoc.Y,0),FVector(targetLoc.X, targetLoc.Y, 0)).Yaw;

		float _fov = angleOf2DVector(currentLoc, targetLoc) - _phi0Value;
		
		float _phi1Value = 0;
		//対象の次のパスが存在しない
		if(!pathDatas.IsValidIndex(next)){
			//角度を変更せずにそのままの角度を設定する
			 _phi1Value = _phi0Value;
		//対象の次のパスが存在する
		}else{
			//対象のパスから対象の次のパスまでの角度を設定する
			FVector2D nextLoc(pathDatas[next].X, pathDatas[next].Y);
			_phi1Value = UKismetMathLibrary::FindLookAtRotation(FVector(targetLoc.X, targetLoc.Y, 0), FVector(nextLoc.X, nextLoc.Y, 0)).Yaw;
			 
		}

		TArray<FVector2D>curve = calcClothoidCurve(accuracy, _phi1Value, _phi0Value, _hValue, _fov);
		
		//仮の高さ設定
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

