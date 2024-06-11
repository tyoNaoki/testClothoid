




// Fill out your copyright notice in the Description page of Project Settings.

#include "clothoidLine.h"
#include "Components/BillboardComponent.h"
#include <iostream>
#include "Kismet/KismetMathLibrary.h"
#include "DrawDebugHelpers.h"

using namespace std;
DEFINE_LOG_CATEGORY_STATIC(Clothoid_Debug_LOG, Error, All);


// Sets default values
AclothoidLine::AclothoidLine()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	billboards.Empty();

	pointMark = CreateDefaultSubobject<UBillboardComponent>(TEXT("BillBoard"));
	SetRootComponent(pointMark);
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

void AclothoidLine::phiSimpson_integral(FPhiSlope f, float a, float b, std::complex<double>* r) {
	double mul = (b - a) * static_cast<double>(1.0 / 6.0);
	*r = mul * (f(a) + static_cast<double>(4.0) * f((a + b) * static_cast<double>(0.5)) + f(b));
}

float AclothoidLine::calc_TurnRate_Newton(float _sign, FCircle &circle, FVector _target)
{
	float error = 1.0;
	float delta = 1;
	float x1 = 0;
	int k = 0;
	int countMax = 30;
	float _centerAngle = circle.centerRotation.Yaw;
	float _radius = circle.radius;
	FVector _center = circle.center;

	while (true) {
		float x2 = x1 + delta;
		float y1 = f_TurnRate(x1,_sign,_centerAngle,_radius, _center,_target);
		float y2 = f_TurnRate(x2, _sign, _centerAngle, _radius, _center, _target);
		if((x2-x1) == 0.0f || (y2-y1)==0.0f){
			return 0.0f;
		}
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

	return x1;
}

float AclothoidLine::calc_TurnRate_Newton2(FCircle& circle, FCircle& circle2)
{
	
	float error = 1.0;
	float delta = 1;
	float x1 = 0;
	int k = 0;
	int countMax = 30;
	float _sign = circle.sign*-1;
	float _centerAngle = circle.centerRotation.Yaw;
	//float _targetCenterAngle = circle2.centerRotation.Yaw;
	float _radius = circle.radius;
	FVector _center = circle.center;

	while (true) {
		float x2 = combineFloat(x1,delta);
		//float x2 = combineFloat(x1, delta);
		UE_LOG(Clothoid_Debug_LOG,Error,TEXT("x2 = %f"),x2);

		
		
		float angle = combineFloat(_centerAngle, x1 * _sign);
		FVector angleLoc = getAngleLocation(angle, _radius, _center);
		float targetAngle = combineFloat(angle,180.0);
		FVector _target = getAngleLocation(targetAngle, circle2.radius, circle2.center);

		float y1 = f_TurnRate(x1, _sign, _centerAngle, _radius, _center, _target);

		angle = combineFloat(_centerAngle, x2 * _sign);
		angleLoc = getAngleLocation(angle, _radius, _center);
		targetAngle = combineFloat(angle, 180.0);
		_target = getAngleLocation(targetAngle, circle2.radius, circle2.center);

		float y2 = f_TurnRate(x2, _sign, _centerAngle, _radius, _center, _target);
		if((x2-x1)==0.0f||(y2-y1)==0.0f){
			break;
		}

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

	return x1;
}

float AclothoidLine::f_TurnRate(float value,float _sign, float _centerAngle, float _radius, FVector _center, FVector _target){
	float combineAngle = combineFloat(_centerAngle, value * _sign);
	
	FVector angleLoc = getAngleLocation(combineAngle,_radius,_center);

	//円弧上の角度の接点から対象のパスまでの角度計算
	float angle_To_Target = UKismetMathLibrary::FindLookAtRotation(angleLoc, _target).Yaw;

	//円弧上の接線の角度計算
	float angle = UKismetMathLibrary::FindLookAtRotation(_center,angleLoc).Yaw;
	float angleOnCircle =  combineFloat(angle,_sign*90);

	return deltaFloat(angle_To_Target,angleOnCircle);
}

FRotator AclothoidLine::getRotFromCircleTangent(FVector center, FVector circleTangent, int sign){

	FRotator rot = UKismetMathLibrary::FindLookAtRotation(center,circleTangent);
	rot = UKismetMathLibrary::ComposeRotators(rot,FRotator(0,90*sign,0));
	return rot;
}

TArray<FVector> AclothoidLine::tripleClothoidCurve(const float phi1, const float phi0, FVector start, FVector goal)
{
	TArray<FVector>clothoidCurve;
	float h = 0.0;
	FSlope slope;
	float theta = UKismetMathLibrary::FindLookAtRotation(start,goal).Yaw;

	float r = FVector::Dist(start,goal);

	if(!calcClothoidParameter(phi1,phi0,theta,r,h,slope)){
		UE_LOG(LogTemp,Error,TEXT("calcClothoidParameter is failed!! [phi1 : %f,phi0 : %f,start : %s,goal : %s]"),phi1,phi0,*start.ToString(),*goal.ToString());
		return clothoidCurve;
	}

	UE_LOG(LogTemp, Error, TEXT("clothoidParam : [h : %f,phi1 : %f,phi0 : %f,phiV : %f,phiU : %f]"),h,UKismetMathLibrary::DegreesToRadians(phi1),UKismetMathLibrary::RadiansToDegrees(slope.phi0), UKismetMathLibrary::RadiansToDegrees(slope.phiV), UKismetMathLibrary::RadiansToDegrees(slope.phiU));

	if(h<=0.0f){
		//UE_LOG(LogTemp, Error, TEXT("calcClothoidParameter is failed!! [phi1 : %f,phi0 : %f,start : %s,goal : %s,h : %f]"), phi1, phi0, *start.ToString(), *goal.ToString(),h);
		return clothoidCurve;
	}

	int num = (int)h;
	clothoidCurve.SetNum(num);
	float stepS = 1.0f / h;
	complex<float> curve;

	for (int i = 0; i < num; ++i) {

		float S = stepS * i;

		complex<float> p;
		simpson_integral(slope, S, S + stepS, &p);
		curve += p;

		float x = static_cast<float>(curve.real());
		float y = static_cast<float>(curve.imag());

		FVector2D clothoidCurve2D = (h * FVector2D(x, y));
		clothoidCurve[i] = start + FVector(clothoidCurve2D, 0);
	}

	return clothoidCurve;
}

bool AclothoidLine::calcClothoidParameter(const float phi1,const float phi0,const float theta,const float r,float& h, FSlope& slope)
{
	FSlope f;
	//始点から対象の制御点に向かう方向に取ったローカル座標に変換するために角度をローカルに変更
	const float localPhi0 = UKismetMathLibrary::DegreesToRadians((phi0-theta));
	//localPhi0 + phiV + phiU = phi1 - theta
	const float localPhi1 = UKismetMathLibrary::DegreesToRadians(phi1-theta);

	//f.phiV = UKismetMathLibrary::DegreesToRadians(phiV);
	//f.phiU = UKismetMathLibrary::DegreesToRadians(phiU);

	//∫[1-0]sin(phi(s)dS) = 0
	if(!newtonMethod(localPhi1, localPhi0, f.phiV, f.phiU,r)){
		return false;
	}

	//h * ∫[1-0](cos(phi(s))dS) = r

	f.phi0 = localPhi0;
	//f.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);

	UE_LOG(LogTemp,Error,TEXT("simpson is %f,phiV is %f,phiU is %f"), simpson_integral_cos(f, 0, 1),UKismetMathLibrary::RadiansToDegrees(f.phiV),UKismetMathLibrary::RadiansToDegrees(f.phiU));
	
	
	float cosP = 0.0f;
	float stepS = 1.0f / 50.0f;

	for (int i = 0; i < 50.0f; ++i) {
		float S = stepS * i;

		std::complex<float> r;
		cosP += simpson_integral_cos(f, S, S + stepS);
	}

	h = r / cosP;
	

	//h =  r / simpson_integral_cos(f, 0, 1);

	slope = f;

	//phi0をグローバルに戻す
	slope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);

	return true;
}

float AclothoidLine::simpson_integral_cos(FSlope f, float a, float b)
{
	float mul = (b - a) * static_cast<float>(1.0 / 6.0);
	return mul * (f.slope_f_Cos(a) + static_cast<float>(4.0) * f.slope_f_Cos((a + b) * static_cast<float>(0.5)) + f.slope_f_Cos(b));
}

float AclothoidLine::simpson_integral_sin(FSlope f,float a,float b)
{
	float mul = (b - a) * static_cast<float>(1.0 / 6.0);
	return mul * (f.slope_f_Sin(a) + static_cast<float>(4.0) * f.slope_f_Sin((a + b) * static_cast<float>(0.5)) + f.slope_f_Sin(b));
}

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

float AclothoidLine::angle_diff2(float theta1, float theta2)
{
	float degree = UKismetMathLibrary::RadiansToDegrees(theta1);
	float degree2 = UKismetMathLibrary::RadiansToDegrees(theta2);
	float deltaValue = deltaFloat(degree,degree2);
	return UKismetMathLibrary::DegreesToRadians(deltaValue);
}

FRotator AclothoidLine::delta_VectorAngle(FVector p1, FVector p2)
{
	FVector zeroV = FVector::ZeroVector;
	FRotator r1 = UKismetMathLibrary::FindLookAtRotation(zeroV,p1);
	FRotator r2 = UKismetMathLibrary::FindLookAtRotation(zeroV,p2);

	return UKismetMathLibrary::NormalizedDeltaRotator(r1,r2);
}

void AclothoidLine::calcCircleInfo(FCircle& circle, FVector targetLocation){ 
	//targetlocationとの数値差分をcircleに加算して修正する
	FVector diffLocation = targetLocation - circle.startCircle;
	circle.startCircle +=diffLocation;
	circle.centerCircle +=diffLocation;
	circle.center += diffLocation;
}

void AclothoidLine::test_circleCheck(FCircle& circle){
	if(check_loopCount+1 == check_CircleIndex){
		test_Circle = circle;
	}
}

float AclothoidLine::get_clothoid_angle(float radius)
{
	double scale = 1; //スケール因子
	double L = scale * UKismetMathLibrary::Sqrt(radius);
	float angle = pow(L,2) / (2 * pow(scale,2));
	return angle;
}

float AclothoidLine::calc_curve_angle(float length, float radius)
{
	//値調整用

	float scale = 1;

	if(radius == 0.0f){
		return 0.0f;
	}

	return (1/radius) * length * scale;
}

FVector AclothoidLine::calcMaxCircleAngle(FCircle& circle, FCircle& circle2)
{
	FVector center = circle.center;
	FVector center2 = circle2.center;
	FRotator center_to_center2 = UKismetMathLibrary::FindLookAtRotation(center,center2);
	FRotator circleAngle = FRotator(0,circle2.sign*90,0);
	FRotator maxRot = UKismetMathLibrary::ComposeRotators(center_to_center2,circleAngle);
	return circle2.center + UKismetMathLibrary::GetForwardVector(maxRot)*circle2.radius;
}

double AclothoidLine::fx(int n, float phi, double x,float fov)
{
	float stepS = 1.0f / n;
	TArray<FVector2D> points;
	FPhiSlope pSlope;
	
	pSlope.phiV = UKismetMathLibrary::DegreesToRadians(x);
	pSlope.phiU = phi - pSlope.phiV;

	//与えられた視野角と一致するか計算

	complex<double> psiP_Vector;
	for (int i = 0; i < n; ++i) {
		double S = stepS * i;

		complex<double> r;
		phiSimpson_integral(pSlope, S, S + stepS, &r);
		psiP_Vector += r;

		float x = static_cast<float>(psiP_Vector.real());
		float y = static_cast<float>(psiP_Vector.imag());
		points.Add(1 * FVector2D(x, y));
	}

	// 2点の座標からラジアンを求める
	double radian = atan2(points.Last().Y - points[0].Y, points.Last().X - points[0].X);

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
	double metorSpeed = turn_speed * 1.852 * 1000 / 3600;
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
		UE_LOG(LogTemp, Verbose, TEXT("fx(%f,%f,%f,%f)"), n, _phi, x1, fov);
		float y2 = fx(n, _phi,x2,fov);
		if(abs(x2 - x1) == 0){
			UE_LOG(Clothoid_Debug_LOG,Error,TEXT("[%d] x2-x1 = %f"),check_loopCount,x2-x1);
			break;
		}

		float diff_x = (y2 - y1) / (x2 - x1);
		float next_x = x1 - y1 / diff_x;
		UE_LOG(LogTemp, Verbose, TEXT("[%d] diff_x(%f) = (y2(%f) - y1(%f)) / (x2(%f) - x1(%f))"),k,diff_x,y2,y1,x2,x1);
		UE_LOG(LogTemp, Verbose, TEXT("[%d] next_x(%f) = x1(%f) - y1(%f) / diff_x(%f)"), k, next_x, x1, y1, diff_x);

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

float AclothoidLine::angle_Vector(float value,float r){
	FVector point = FVector(r, value, 0);
	FVector target = FVector(r, 0, 0);
	FVector start = FVector::ZeroVector;

	// 2点の座標からラジアンを求める
	double radian = atan2(point.Y - start.Y, point.X - start.X);
	double targetRadian = atan2(target.Y - start.Y, target.X - start.X);

	//degree = UKismetMathLibrary::RadiansToDegrees(radian)
	double diff = angle_diff(radian, targetRadian);
	return UKismetMathLibrary::RadiansToDegrees(diff);
}



bool AclothoidLine::newtonMethod(const float localPhi1, const float localPhi0, float& phiV, float& phiU,float r)
{
	float error = 0.01;
	//float error = 0.001;
	float delta = 0.001;
	//float delta = 0.01;
	float x1 = 0;
	int k = 0;
	int countMax = 30;

	FSlope f;
	f.phi0 = localPhi0;
	float tempPhi = angle_diff(localPhi1,localPhi0);
	//float tempPhi = localPhi1-localPhi0;
	float fov = UKismetMathLibrary::RadiansToDegrees(tempPhi);
	//fov = 100.0f;

	while (true) {
		float y1 = fx(r,tempPhi,x1,fov);
		UE_LOG(LogTemp,Verbose,TEXT("fx(%f,%f,%f,%f)"),r,tempPhi,x1,fov);

		float x2 = x1 + delta;
		float y2 = fx(r,tempPhi,x2,fov);
		if (abs(x2 - x1) == 0.0f || abs(y2 - y1) == 0.0f) {
			UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d] x2-x1 = %f, y2 - y1 = %f"), check_loopCount, x2 - x1, y2 - y1);
			break;
		}

		if (std::fabs(y1) < error || k >= countMax) {
			break;
		}

		float diff_x = (y2 - y1) / (x2 - x1);
		UE_LOG(LogTemp, Verbose, TEXT("[%d] diff_x(%f) = (y2(%f) - y1(%f)) / (x2(%f) - x1(%f))"), k, diff_x, y2, y1, x2, x1);

		//float diff_x = angle_diff(y2,y1) / angle_diff(x2,x1);

		float next_x = x1 - y1 / diff_x;

		UE_LOG(LogTemp, Verbose, TEXT("[%d] next_x(%f) = x1(%f) - y1(%f) / diff_x(%f)"), k, next_x, x1, y1, diff_x);

		x1 = next_x;

		//UE_LOG(LogTemp, Log, TEXT("[%d] x1 = %f , y1 = %f"), k, UKismetMathLibrary::RadiansToDegrees(x1), y1);
		k += 1;
	}

	/*
	while (true) {
		f.phiV = x1;
		f.phiU = tempPhi - x1;
		float y1 = simpson_integral_sin(f,0,1);

		//y1 = angle_Vector(y1,r);

		float x2 = x1 + delta;
		f.phiV = x2;
		f.phiU = tempPhi - x2;
		float y2 = simpson_integral_sin(f, 0, 1);

		//y2 = angle_Vector(y2,r);

		//Debug
		float degree1 = UKismetMathLibrary::RadiansToDegrees(x1);
		float degree2 = UKismetMathLibrary::RadiansToDegrees(x2);
		//

		if (abs(x2 - x1) == 0.0f || abs(y2 - y1) == 0.0f) {
			UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d] x2-x1 = %f, y2 - y1 = %f"), check_loopCount, x2 - x1,y2 - y1);
			break;
		}

		if (std::fabs(y1) < error || k >= countMax) {
			break;
		}
		
		float diff_x = (y2 - y1) / (x2 - x1);
		UE_LOG(LogTemp, Verbose,TEXT("[%d] diff_x(%f) = (y2(%f) - y1(%f)) / (x2(%f) - x1(%f))"),k,diff_x,y2,y1,x2,x1);

		//float diff_x = angle_diff(y2,y1) / angle_diff(x2,x1);

		float next_x = x1 - y1 / diff_x;
		
		UE_LOG(LogTemp, Verbose, TEXT("[%d] next_x(%f) = x1(%f) - y1(%f) / diff_x(%f)"),k,next_x,x1,y1,diff_x);

		//float next_x = angle_diff(x1,(y1 / diff_x));

		//Debug
		float degreeDiff = UKismetMathLibrary::RadiansToDegrees(angle_diff2(x2,x1));
		//
	
		//float next_x = x1 - y1 / diff_x;
		//float next_x = angle_diff2(x1,y1) / diff_x;

		x1 = next_x;
		x1 = UKismetMathLibrary::FClamp(x1,UKismetMathLibrary::DegreesToRadians(-720), UKismetMathLibrary::DegreesToRadians(720));

		UE_LOG(LogTemp, Log, TEXT("[%d] x1 = %f , y1 = %f"), k, UKismetMathLibrary::RadiansToDegrees(x1),y1);
		k += 1;
	}
	*/

	//UE_LOG(LogTemp, Log, TEXT("(result : phiV = %f(%f), phiU = %f(%f),phi1 : %f(%f), phi0 : %f(%f)),count %d"),
		//x1,UKismetMathLibrary::RadiansToDegrees(x1),tempPhi-x1,UKismetMathLibrary::RadiansToDegrees((tempPhi-x1)),localPhi1,UKismetMathLibrary::RadiansToDegrees(localPhi1),localPhi0,UKismetMathLibrary::RadiansToDegrees(localPhi0), k);
	UE_LOG(LogTemp, Log, TEXT("(result : phiV = %f(%f), phiU = %f(%f),phi1 : %f(%f), phi0 : %f(%f)),count %d"),
		UKismetMathLibrary::DegreesToRadians(x1),x1,tempPhi - UKismetMathLibrary::DegreesToRadians(x1), UKismetMathLibrary::RadiansToDegrees(tempPhi) - x1, localPhi1, UKismetMathLibrary::RadiansToDegrees(localPhi1), localPhi0, UKismetMathLibrary::RadiansToDegrees(localPhi0), k);

	if (k >= countMax) {
		UE_LOG(LogTemp, Error, TEXT("Failed to converge within %d iterations!!"),countMax);
		return false;
	}
	
	phiV = UKismetMathLibrary::DegreesToRadians(x1);
	//phiU = angle_diff2(tempPhi,x1);
	phiU = tempPhi-phiV;

	return true;
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


float AclothoidLine::getMiddleAngle_gimbal(float angle1, float angle2, bool sign)
{
	float value = (angle1 - angle2)/2;

	return sign ? combineFloat(angle1,value) : deltaFloat(angle1,value);
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

TArray<FVector> AclothoidLine::calcCircleLocations(float startAngle, float endAngle,float radius,FVector curve_start,FVector curve_end,float sign)
{
	TArray<FVector>results;

	float zDiff = curve_end.Z - curve_start.Z;
	
	float angleLength = deltaFloat(UKismetMathLibrary::DegreesToRadians(endAngle), UKismetMathLibrary::DegreesToRadians(startAngle));
	float test_angleLength = deltaFloat(startAngle,endAngle);
	//angleLength = UKismetMathLibrary::RadiansToDegrees(angleLength);
	angleLength = test_angleLength;
	UE_LOG(Clothoid_Debug_LOG,Error,TEXT("[%d] angleLength = %f angleDiff = %f,startAngle = %f ,endAngle = %f"),check_loopCount,angleLength, test_angleLength,startAngle,endAngle);


	int num = UKismetMathLibrary::Abs(angleLength)* ((float)arc_accuracy / 100.0);
	if(num < 1){
		results.Add(curve_start);
		return results;
	}

	float step = 1.0f / num;

	FVector center = FVector::ZeroVector;
	FVector startAngleLoc = getAngleLocation(startAngle, radius, center);
	//FVector endAngleLoc = getAngleLocation(endAngle, radius, center);
	//FRotator rot1 = UKismetMathLibrary::FindLookAtRotation(startAngleLoc,endAngleLoc);
	//UE_LOG(Clothoid_Debug_LOG,Error,TEXT("sign : %f"),sign);

	if (sign == 1) {
		angleLength = -std::abs(angleLength);
	}
	else {
		angleLength = std::abs(angleLength);
	}

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

TArray<FVector> AclothoidLine::calcCircleLocations2(float startAngle, float endAngle, float radius, FVector curve_center, FVector curve_start, FVector curve_end, float sign)
{
	TArray<FVector> points;
	float radStartAngle = UKismetMathLibrary::DegreesToRadians(startAngle);
	float radEndAngle = UKismetMathLibrary::DegreesToRadians(endAngle);
	int num = UKismetMathLibrary::Abs(deltaFloat(startAngle,endAngle)) * ((float)arc_accuracy / 100.0);
	double deltaAngle = (radEndAngle - radStartAngle);
	float step = 1.0f / num;

	if (sign == 1) {
		deltaAngle = -std::abs(deltaAngle);
	}
	else{
		deltaAngle = std::abs(deltaAngle);
	}



	for (int i = 0; i <= num; ++i) {
		float s = i * step;
		double angle = radStartAngle + deltaAngle * s;
		FVector tempCenter = FVector::ZeroVector;
		FVector startAngleLoc = getAngleLocation(startAngle, radius, tempCenter);
		FVector arcPoint = getAngleLocation(UKismetMathLibrary::RadiansToDegrees(angle), radius, tempCenter);
		FVector loc;
		//loc.X = tempCenter.X + radius * cos(angle);
		//loc.Y = tempCenter.Y + radius * sin(angle);
		//loc.Z = tempCenter.Z;
		loc.X = curve_start.X + arcPoint.X - startAngleLoc.X;
		loc.Y = curve_start.Y + arcPoint.Y - startAngleLoc.Y;
		loc.Z = curve_start.Z;

		points.Add(loc);
	}

	return points;
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

TArray<FVector> AclothoidLine::calcClothoidCurve(float phi1, float phi0,float length,float fov,FVector controllLoc,FVector curveStartLoc){
	
	int num_CalcClothoid = 50;
    double stepS = 1.0f / num_CalcClothoid;
	const int n = length;
	//float iota = length;
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
	float rad_phi1 = UKismetMathLibrary::DegreesToRadians(phi1);
	float rad_phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
	float diff_phi = angle_diff(rad_phi1, rad_phi0);
	//float diff_phi = rad_phi1-rad_phi0;
	//UE_LOG(Clothoid_Debug_LOG,Error,TEXT("[phi1(%f) ~ phi0(%f)]diff_phi is % f"),phi1,phi0, (diff_phi));

	//float _tempPhi = UKismetMathLibrary::DegreesToRadians(phi1 - phi0);
	//float _tempPhi = diff_phi;
	//UE_LOG(LogTemp, Error, TEXT("phi1 is %f,phi0 is %f"), phi1,phi0);

	//挟み撃ち法で計算
	
	//-1.1491665778430064a
	UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d]n = %d , diff_phi is %f, fov = %f"), check_loopCount,n,diff_phi,fov);
	phiSlope.phiV = UKismetMathLibrary::DegreesToRadians(getPhi_newtonMethod(n, diff_phi,fov));
	//phiSlope.phiV = 0.0f;
   // phiSlope.phiU = UKismetMathLibrary::DegreesToRadians(phi1 - phi0) - phiSlope.phiV;
	phiSlope.phiU = diff_phi - phiSlope.phiV;
	//phiSlope.phiU = diff_phi - phiSlope.phiV;

	
	psiPoints.SetNum(num_CalcClothoid);
	
    complex<double> psiP_Vector;
    for (int i = 0; i < num_CalcClothoid; ++i) {
        double S = stepS * i;

        complex<double> r;
        phiSimpson_integral(phiSlope, S, S + stepS, &r);
        psiP_Vector += r;

        float l_x = static_cast<float>(psiP_Vector.real());
        float l_y = static_cast<float>(psiP_Vector.imag());
		
		psiPoints[i] = (1 * FVector2D(l_x, l_y));
        //psiPoints.Add(1 * FVector2D(l_x, l_y));
    }
	
    float lamda = UKismetMathLibrary::Distance2D(psiPoints.Last(),psiPoints[0]);
	
    float h = length / lamda;

	UE_LOG(LogTemp,Verbose,TEXT("[%d] : h = %f, length = %f / lamda = %f"),check_loopCount,h,length,lamda);

    FSlope cSlope;
    cSlope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
    cSlope.phiV = phiSlope.phiV;
    cSlope.phiU = phiSlope.phiU;
	UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d] : phiSlope.phiV = %f, phiSlope.phiU = %f"), check_loopCount, phiSlope.phiV, phiSlope.phiU);

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
		
		results[i] = FVector(clothoidCurve.X + controllLoc.X ,clothoidCurve.Y + controllLoc.Y,controllLoc.Z + zDiff * S);
    }

	return results;
}

TArray<FVector> AclothoidLine::clothoidCurve(int num,float h,float phi0,float phiV,float phiU,FVector controllLoc)
{
	TArray<FVector>result;
	result.SetNum(num);
	float stepS = 1.0f/num;
	FSlope slope;
	
	slope.phi0 = UKismetMathLibrary::DegreesToRadians(phi0);
	slope.phiV = UKismetMathLibrary::DegreesToRadians(phiV);
	slope.phiU = UKismetMathLibrary::DegreesToRadians(phiU);
	complex<float> clothoid;

	for (int i = 0; i < num; ++i) {
		float S = stepS * i;

		complex<float> r;
		simpson_integral(slope,S, S + stepS, &r);
		clothoid += r;

		float l_x = static_cast<float>(clothoid.real());
		float l_y = static_cast<float>(clothoid.imag());

		FVector2D clothoidCurve2D = (h * FVector2D(l_x, l_y));
		result[i] = controllLoc + FVector(clothoidCurve2D,0);
	}

	return result;
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

float AclothoidLine::combineFloat(float yaw1,float yaw2)
{
	FRotator rot1 = FRotator(0,yaw1,0);
	FRotator rot2 = FRotator(0, yaw2, 0);
	return UKismetMathLibrary::ComposeRotators(rot1,rot2).Yaw;
}
float AclothoidLine::deltaFloat(float yaw1, float yaw2)
{
	FRotator rot1 = FRotator(0, yaw1, 0);
	FRotator rot2 = FRotator(0, yaw2, 0);
	return UKismetMathLibrary::NormalizedDeltaRotator(rot1, rot2).Yaw;
}

float AclothoidLine::calc_TurnRate2(FVector current, FVector center, FVector target)
{
	FRotator angle1 = UKismetMathLibrary::FindLookAtRotation(center, current);
	FRotator angle2 = UKismetMathLibrary::FindLookAtRotation(center, target);
	float angleDiff = UKismetMathLibrary::NormalizedDeltaRotator(angle2, angle1).Yaw;
	return angleDiff;
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
	int splineNum = pathDatas.Num();

	FVector tempVec = pathDatas[target].p_controll_Loc - currentLoc;
	float tempAngle = UKismetMathLibrary::DegAtan2(tempVec.Y, tempVec.X);

	pre_Circle = FCircle();

	results.Add(currentLoc);

	for(; current < splineCount;){
		check_loopCount = current;
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
		FVector arc_center;
		float arc_sign;

		FRotator arc_Rot;

		//旋回半径
		float radius = calcTurnRadius(speed, turningPerformance, scale_calcTurnRadius);
		//始点角度
		float startAngle = 0;
		//終点角度
		float endAngle = 0;

		float signVal;

		//円弧上の接線角度
		float _tangentAngleOnCircle = 0;

		//float _phi0Value = pathDatas[current].phi0;
		//float _phi1Value = pathDatas[current].phi1;

		//次のパスがある場合
		if(pathDatas.IsValidIndex(next)){
			
			float distanceToTarget = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);
			float distanceToNext = UKismetMathLibrary::Distance2D(targetLocXY,FVector2D(pathDatas[next].p_controll_Loc.X, pathDatas[next].p_controll_Loc.Y));
			//旋回半径がパス間の距離の半分をこえた場合、パス間の距離の半分を旋回半径にする
			if((distanceToTarget / 2.1)<=radius){
				radius = distanceToTarget / 2;
			}else if((distanceToNext / 2.1) <= radius){
				radius = distanceToNext / 2;
			}

			float r1 = UKismetMathLibrary::FindLookAtRotation(targetLoc, currentLoc).Yaw;
			float r2 = UKismetMathLibrary::FindLookAtRotation(targetLoc, pathDatas[next].p_controll_Loc).Yaw;
			FRotator angle_Middle = FRotator(0, getMiddleAngle(r1, r2), 0);

			arc_Rot = UKismetMathLibrary::ComposeRotators(angle_Middle, FRotator(0, 180, 0));
			float angle_threePoint = getThreePointAngle(currentLoc, targetLoc, pathDatas[next].p_controll_Loc);
			signVal = UKismetMathLibrary::SignOfFloat(angle_threePoint);
			//turn_rateの大きさで始点角度が決まる。（turn_rate : 0<=X<=90)
			//ここのturn_rateは始点角度の変化量
			//実装予定(turn_rateを始点角度変化量に変更、終点角度変化量変数を作成する必要アリ)
			
			//angle_Middle.Yaw - r1
			
			//float angle_circle = UKismetMathLibrary::NormalizedDeltaRotator(composeRot, deltaRot).Yaw;
			
			//turn_rate_start = calc_TurnRate(currentLoc,targetLoc,pathDatas[next].p_controll_Loc);
			
			FVector center = targetLoc + UKismetMathLibrary::GetForwardVector(angle_Middle) * radius;
			arc_center = center;
			float centerAngle = arc_Rot.Yaw;
			
			if(pre_Circle.isPoint){
				//circles[current - 1].endLoc_OnCircle;
				//float turn_rate_startMax = calc_TurnRate_Newton(signVal*-1, centerAngle, radius, center, currentLoc);
				float turn_rate_startMax = 0.0f;
				//UE_LOG(Clothoid_Debug_LOG, Error,TEXT("turn_rate_startMax is %f"),turn_rate_startMax);
				turn_rate_start = turn_rate_startMax;
				//float turn_rate_endMax = calc_TurnRate_Newton(signVal, centerAngle, radius, center, pathDatas[next].p_controll_Loc);
				float turn_rate_endMax = 0.0f;
				//UE_LOG(Clothoid_Debug_LOG, Error, TEXT("turn_rate_endMax is %f"), turn_rate_endMax);
				turn_rate_end = turn_rate_endMax;

			}else{
				//pre_circle上の点を対象に行う必要あり
				//仮として、対象をパス本体にしている
				//float turn_rate_startMax = calc_TurnRate_Newton(signVal * -1, centerAngle, radius, center, currentLoc);
				float turn_rate_startMax = 0.0f;
				//UE_LOG(Clothoid_Debug_LOG, Error, TEXT("turn_rate_startMax is %f"), turn_rate_startMax);
				turn_rate_start = turn_rate_startMax;
				//float turn_rate_endMax = calc_TurnRate_Newton(signVal, centerAngle, radius, center, pathDatas[next].p_controll_Loc);
				float turn_rate_endMax = 0.0f;
				//UE_LOG(Clothoid_Debug_LOG, Error, TEXT("turn_rate_endMax is %f"), turn_rate_endMax);
				turn_rate_end = turn_rate_endMax;
				
			}
			
			//turn_rate_start = calc_TurnRate(currentLoc, targetLoc,pathDatas[next].p_controll_Loc);
			//turn_rate_end = calc_TurnRate(pathDatas[next].p_controll_Loc, targetLoc, currentLoc);

			//turn_rate_end = calc_TurnRate2(targetLoc, center, pathDatas[next].p_controll_Loc);

			//turn_rate_end = test_turnRateEnd;

			FRotator startRot = FRotator(0, signVal * turn_rate_start, 0);
			
			FRotator endRot = FRotator(0, -1 * signVal * turn_rate_end, 0);

			startAngle = UKismetMathLibrary::NormalizedDeltaRotator(arc_Rot, startRot).Yaw;
			endAngle = UKismetMathLibrary::NormalizedDeltaRotator(arc_Rot, endRot).Yaw;

			//旋回半径の円弧の始まり
			arc_start = getAngleLocation(startAngle, radius, center);

			//FVector arcstart1 = calcStartLocationOnArc(currentLoc, radius, center, signVal);
			
			//float startAngle1 = UKismetMathLibrary::FindLookAtRotation(center, arcstart1).Yaw;

			arc_end = getAngleLocation(endAngle, radius, center);

			//高さ計算
			float arcLen = calculateArcLength(radius, startAngle, endAngle);
			float curveLen = distanceToTarget + arcLen;
			float zDiff = targetLoc.Z - currentLoc.Z;

			arc_start.Z = ((distanceToTarget / curveLen) * zDiff) + currentLoc.Z;
			arc_end.Z = targetLoc.Z;

			//円弧上の接線の角度を取得する
			_tangentAngleOnCircle = UKismetMathLibrary::ComposeRotators(UKismetMathLibrary::FindLookAtRotation(center, arc_start), FRotator(0, signVal * 90, 0)).Yaw;

			pre_Circle = FCircle(center,radius,arc_Rot,startAngle,endAngle,signVal);
			pre_Circle.centerCircle = getAngleLocation(arc_Rot.Yaw,radius,center);
			pre_Circle.startCircle = getAngleLocation(startAngle, radius, center);
			pre_Circle.endCircle = getAngleLocation(endAngle, radius, center);
			pre_Circle.isPoint = false;
			
			
			//arc_sign = UKismetMathLibrary::SignOfFloat(deltaFloat(UKismetMathLibrary::FindLookAtRotation(pre_Circle.startCircle, pre_Circle.centerCircle).Yaw, UKismetMathLibrary::FindLookAtRotation(pre_Circle.startCircle, pre_Circle.endCircle).Yaw));

			arc_sign = signVal*-1;

			//controll_p_Location = getAngleLocationFromThreepoint(turn_rate,currentLoc,targetLoc,pathDatas[next].p_controll_Loc,radius);
		}else{//次のパスがない場合
			//対象のパス座標に制御点に設定する
			arc_start = targetLoc;
			arc_end = targetLoc;
			
			//円弧が無いので現在のパス座標から対象のパス座標への角度をそのまま制御点に設定する
			FVector vec = targetLoc - currentLoc;
			arc_Rot = UKismetMathLibrary::FindLookAtRotation(currentLoc,targetLoc);
			_tangentAngleOnCircle = UKismetMathLibrary::DegAtan2(vec.Y, vec.X);
			//_tangentAngleOnCircle = arc_Rot.Yaw;
			//始点角度、終点角度はそのまま0で設定

			signVal = 0;
			arc_sign = 0;
			arc_center = targetLoc;

			pre_Circle = FCircle(targetLoc, arc_Rot,_tangentAngleOnCircle,_tangentAngleOnCircle,signVal);
			//pre_Circle.isPoint = false;
		}

		FVector vec_tangent = arc_start - currentLoc;

		arc_start_XY = FVector2D(arc_start.X, arc_start.Y);

		//距離取得
		//float _hValue = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);
		float _hValue = UKismetMathLibrary::Distance2D(arc_start_XY, currentLocXY);
		UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[current : %d] _hvalue : %f"),current,_hValue);
		
		int num = _hValue * ((float)accuracy / 100.0);
		//int num = 1000;
		//int num = accuracy;
		//float stepS = 1.0f / num;

		//_hValue = 

		//このphi0は、前の制御点のphi1を考慮せず、計算している
		//実装予定
		//float _phi0Value = convert_Angle_to_controllAngle(UKismetMathLibrary::DegAtan2(vec_tangent.Y, vec_tangent.X));
		float _phi0Value = UKismetMathLibrary::DegAtan2(vec_tangent.Y, vec_tangent.X);

		//float _phi1Value = convert_Angle_to_controllAngle(_tangentAngleOnCircle);
		float _phi1Value = _tangentAngleOnCircle;

		UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d ~ %d] _phi0Value is %f , _phi1Value is %f"),target,current,_phi0Value,_phi1Value );

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
		float _fov = deltaFloat(angleOf2DVector(currentLocXY, arc_start_XY),_phi0Value);
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
		if(num<1){
			num = 1;
			UE_LOG(Clothoid_Debug_LOG,Error,TEXT("num < 1 !!"));
		}

		TArray<FVector>curve = calcClothoidCurve(_phi1Value, _phi0Value, _hValue, _fov, currentLoc, arc_start);

		//円弧の座標取得
		//TArray<FVector>tangent_circles = calcCircleLocations2(startAngle,endAngle,radius,arc_center,curve.Last(),arc_end,arc_sign);

		TArray<FVector>tangent_circles = calcCircleLocations(startAngle, endAngle, radius,curve.Last(), arc_end, arc_sign);
		
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

//テスト用
TArray<FVector> AclothoidLine::calcClothoidSpline2(UPARAM(ref)TArray<FClothoidPath>& pathDatas)
{
	TArray<FVector>results;

	if (pathDatas.Num() <= 1) {
		UE_LOG(LogTemp, Error, TEXT("TArray.Num() is %d"), pathDatas.Num());
		return results;
	}

	//現在速度設定
	float preSpeed = currentSpeed;
	FRotator preRotation = FRotator();
	preRotation.Yaw = pathDatas[0].phi0;

	UE_LOG(Clothoid_Debug_LOG, Error, TEXT("preRotation is %f"), preRotation.Yaw);

	const int splineCount = pathDatas.Num() - 1;

	const float s = 0.5f;

	int current = 0;
	int target = 1;
	int next = 2;
	FVector currentLoc = pathDatas[current].p_controll_Loc;
	FVector2D currentLocXY = FVector2D(currentLoc.X, currentLoc.Y);
	FVector targetLoc;
	FVector2D targetLocXY;

	/*
	if (!clothoidInitiallize) {
		if (GetOwner()) {
			//デバッグモード:始点のクロソイド初期角度を任意の角度に設定する
			//通常時:現在の機体の角度を始点の初期角度に設定
			if (useDebugCurrentRotation) {
				UE_LOG(Clothoid_Debug_LOG, Error, TEXT("currentRotation is %s"), *currentRotation.ToString());
				UE_LOG(Clothoid_Debug_LOG, Error, TEXT("debugMode is true"));
				pre_Rotation = currentRotation;
			}else {
				currentRotation = GetOwner()->GetActorRotation();
			}
		}
		else {
			UE_LOG(LogTemp, Error, TEXT("GetOwner() is Null"));
			return results;
		}
		clothoidInitiallize = true;
	}
	*/

	if (!clothoidInitialize) {
		results.Add(currentLoc);
		clothoidInitialize = true;
	}

	for (; current < splineCount;) {
		check_loopCount = current;

		//曲線長(L)
		float curveLength = 0;

		//初期角度
		float turn_rate_start = 0;

		//終点角度
		float turn_rate_end = 0;

		//次の制御点の座標設定
		targetLoc = pathDatas[target].p_controll_Loc;
		targetLocXY = FVector2D(targetLoc.X, targetLoc.Y);

		//旋回角度
		float turnnningAngle = 1;

		//クロソイド曲線配列
		TArray<FVector>curve;

		//次のパスがある場合
		if (pathDatas.IsValidIndex(next)) {
			//FVector2D nextLocXY = FVector2D(pathDatas[next].X, pathDatas[next].Y);
			//FVector nextLoc = pathDatas[next];

			const FRotator targetRot = UKismetMathLibrary::FindLookAtRotation(currentLoc, targetLoc);

			//FRotator nextRot= UKismetMathLibrary::FindLookAtRotation(targetLoc,nextLoc);
			//距離取得
			const float _hValue = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);

			const float _phi0Value = preRotation.Yaw;
			const float _phi1Value = angleOf2DVector(currentLocXY, targetLocXY);

			UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d ~ %d] _phi0Value is %f , _phi1Value is %f"), current, target, _phi0Value, _phi1Value);

			//視野角計算
			const float _fov = deltaFloat(_phi1Value, _phi0Value);

			//クロソイド曲線を生成
			TArray<FVector> clothoidCurve = calcClothoidCurve(_phi1Value, _phi0Value, _hValue, _fov, FVector(currentLocXY, 0), FVector(targetLocXY, 0));

			//高さ計算と当たり判定を両方行う
			//isNotObstale使用する

			const float curveHeight = targetLoc.Z - currentLoc.Z;

			const float stepS = 1.0f / _hValue;
			for (int i = 0; i < _hValue; i++) {
				float S = stepS * i;
				clothoidCurve[i].Z = curveHeight * S + currentLoc.Z;
			}

			curve.Append(clothoidCurve);

			preControllP = true;
			preRotation = targetRot;

		}
		else {//次のパスがない場合
		//対象のパス座標に制御点に設定する

			const FRotator targetRot = UKismetMathLibrary::FindLookAtRotation(currentLoc, targetLoc);

			//距離取得
			const float _hValue = UKismetMathLibrary::Distance2D(targetLocXY, currentLocXY);

			const int num = _hValue * 1;

			const float _phi0Value = preRotation.Yaw;

			//phi1Valueをphi0Valueの角度に応じて直線で受け取れるようにする
			const float _phi1Value = targetRot.Yaw;

			UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d ~ %d] _phi0Value is %f , _phi1Value is %f"), current, target, _phi0Value, _phi1Value);

			//視野角計算edr
			const float _fov = deltaFloat(_phi1Value, _phi0Value);

			TArray<FVector>clothoidCurve = calcClothoidCurve(_phi1Value, _phi0Value, _hValue, _fov, FVector(currentLocXY, 0), FVector(targetLocXY, 0));

			const float curveHeight = targetLoc.Z - currentLoc.Z;

			const float stepS = 1.0f / num;

			for (int i = 0; i < num; i++) {
				float S = stepS * (i + 1);
				clothoidCurve[i].Z = curveHeight * S + currentLoc.Z;
			}

			//UE_LOG(Clothoid_Debug_LOG, Error, TEXT("[%d] clothoidCurve.Last() is %s"), current, *clothoidCurve.Last().ToString());

			preControllP = true;

			curve.Append(clothoidCurve);

			preRotation = targetRot;

		}

		results.Append(curve);

		//結果の最終座標を次のクロソイド曲線計算のスタート地点に設定
		currentLoc = results.Last();
		currentLocXY = FVector2D(currentLoc.X, currentLoc.Y);

		//UE_LOG(Clothoid_Debug_LOG,Error,TEXT("currentSpeed = %f"), currentSpeed);

		curve.Empty();
		current++;
		target++;
		next++;
	}

	return results;
}

void AclothoidLine::test_Function(){
	
	TFunctionRef<double(double,double)> f = [](double x,double y)
	{
		return x * x * x + y * y * y - 9.0 * x * y + 27.0;
	};
	double testX = 0.0f;
	double testY = 0.0f;
	
	const double epsilon = 0.00001; // 収束判定の閾値
	const int maxIterations = 100; // 最大反復回数

	for (int i = 0; i < maxIterations; ++i) {
		double fx = (f(testX + epsilon, testY) - f(testX, testY)) / epsilon;
		double fy = (f(testX, testY + epsilon) - f(testX, testY)) / epsilon;

		double det = fx * fx + fy * fy;
		double c1 = -fx / det;
		double c2 = -fy / det;

		testX += c1;
		testY += c2;

		if (fabs(c1) < epsilon && fabs(c2) < epsilon) {
			break; // 収束判定
		}
	}

	UE_LOG(LogTemp,Warning,TEXT("testX = %f,testY = %f"),static_cast<float>(testX),static_cast<float>(testY));

	return;
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

