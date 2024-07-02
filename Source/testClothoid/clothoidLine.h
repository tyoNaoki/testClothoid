




// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <complex>
#include <functional>
#include <iterator>
#include <numeric>
#include "Containers/Map.h"
#include "clothoidLine.generated.h"

class UBillboardComponent;

USTRUCT()
struct FSlope {
	GENERATED_USTRUCT_BODY()

	//初期角度
	float phi0;
	//初期曲率
	float phiV;
	//曲率変化率
	float phiU;

	//phi0+phi1*s+phi*s^2
	float phi(float _phi0, float _phiV, float _phiU, float S) {
		return _phi0 + _phiV * S + _phiU * powf(S,2.0);
	}

	//e^j*phi(param)
	std::complex<float> slope_f(float _phi0, float _phiV, float _phiU, float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi(_phi0, _phiV, _phiU, S));
	}

	//cos(phi)+j*sin(phi)
	float slope_f_Cos(float S) {
		return cosf(phi(phi0, phiV, phiU, S));
	}

	float slope_f_Sin(float S){
		return sinf(phi(phi0, phiV, phiU, S));
	}

	std::complex<float> operator()(float S) {
		return slope_f(phi0, phiV, phiU, S);
	}
};

USTRUCT()
struct FPhiSlope {
	GENERATED_USTRUCT_BODY()

	//初期曲率
	float phiV;
	//曲率変化率
	float phiU;

	std::complex<double> slope_f(float _phiV, float _phiU, float S) {
		std::complex<double> j(0.0f, 1.0f);
		return std::exp(j * (double)(phiV * S + phiU * (S * S)));
	}

	std::complex<double> operator()(float S) {
		return slope_f(phiV, phiU, S);
	}
};

USTRUCT(BlueprintType)
struct FDouble_ClothoidParameter
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float s1 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float k0 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float k1 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float phi1 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float phi0 = 0;

	UPROPERTY()
		float h = 100;

	UPROPERTY()
		float a12 = 0;

	UPROPERTY()
		float a22 = 0;

	UPROPERTY()
		float a10 = 0;

	UPROPERTY()
		float a11 = 0;

	UPROPERTY()
		float a20 = 0;

	UPROPERTY()
		float a21 = 0;

	FDouble_ClothoidParameter() = default;

	//0<s1<s2<1

	/*
	FTriple_ClothoidParameter(float _s1, float _s2)
		: s1(_s1), s2(_s2){
	}
	*/
	//a10,a11,a12,a22,a32,h

	std::complex<float> slope_f_phi_1(float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi_1(S));
	}

	std::complex<float> slope_f_phi_1_sin(float S) {
		return sinf(phi_1(S));
	}

	std::complex<float> slope_f_phi_2(float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi_2(S));
	}

	std::complex<float> slope_f_phi_2_sin(float S) {
		return sinf(phi_2(S));
	}


	float phi_1(float s) {
		return a10 + a11 * s + a12 * pow(s, 2);
	}

	float phi_2(float s) {
		return a20 + a21 * (s - s1) + a22 * pow(s - s1, 2);
	}

	void simpson_integral_phi1(float a, float b, std::complex<float>* r)
	{
		float mul = (b - a) * static_cast<float>(1.0 / 6.0);
		*r = mul * (slope_f_phi_1(a) + static_cast<float>(4.0) * slope_f_phi_1((a + b) * static_cast<float>(0.5)) + slope_f_phi_1(b));
	}

	void simpson_integral_phi2(float a, float b, std::complex<float>* r)
	{
		float mul = (b - a) * static_cast<float>(1.0 / 6.0);
		*r = mul * (slope_f_phi_2(a) + static_cast<float>(4.0) * slope_f_phi_2((a + b) * static_cast<float>(0.5)) + slope_f_phi_2(b));
	}

	void simpson_integral_phi1_sin(float a, float b, std::complex<float>* r)
	{
		float mul = (b - a) * static_cast<float>(1.0 / 6.0);
		*r = mul * (slope_f_phi_1_sin(a) + static_cast<float>(4.0) * slope_f_phi_1_sin((a + b) * static_cast<float>(0.5)) + slope_f_phi_1_sin(b));
	}

	void simpson_integral_phi2_sin(float a, float b, std::complex<float>* r)
	{
		float mul = (b - a) * static_cast<float>(1.0 / 6.0);
		*r = mul * (slope_f_phi_2_sin(a) + static_cast<float>(4.0) * slope_f_phi_2_sin((a + b) * static_cast<float>(0.5)) + slope_f_phi_2_sin(b));
	}
};

USTRUCT(BlueprintType)
struct FTriple_ClothoidParameter
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float s1 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float s2 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float startAngle = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float lastAngle = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float curveture0 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float curveture1 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float h = 100;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float a12 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float a22 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "parameter")
		float a32 = 0;

	UPROPERTY()
		float a10 = 0;

	UPROPERTY()
		float a11 = 0;

	UPROPERTY()
		float a20 = 0;

	UPROPERTY()
		float a21 = 0;

	UPROPERTY()
		float a30 = 0;

	UPROPERTY()
		float a31 = 0;

	FTriple_ClothoidParameter()= default;

	//0<s1<s2<1
	/*
	FTriple_ClothoidParameter(float _s1,float _s2,float _a10,float _a11,float _a12,float _a22,float _a32)
	: s1(_s1),s2(_s2),a10(_a10),a11(_a11),a12(_a12),a22(_a22),a30(_a32){
		a20 = a10 + a11*s1 + a12 * pow(s1,2);
		a21 = a11 + 2*a12*s1;
	}
	*/

	/*
	FTriple_ClothoidParameter(float _s1, float _s2)
		: s1(_s1), s2(_s2){
	}
	*/
	//a10,a11,a12,a22,a32,h

	std::complex<float> slope_f_phi_1(float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi_1(S));
	}

	std::complex<float> slope_f_phi_2(float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi_2(S));
	}

	std::complex<float> slope_f_phi_3(float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi_3(S));
	}

	float phi_1(float s){
		return a10 + a11 * s + a12 * pow(s,2);
	}

	float phi_2(float s){
		return a20 + a21 * (s - s1) + a22 * pow(s - s1,2);
	}

	float phi_3(float s) {
		return a30 + a31 * (s - s2) + a32 * pow(s - s2, 2);
	}

};

USTRUCT(BlueprintType)
struct FCircle
{
	GENERATED_USTRUCT_BODY()

	FCircle(FVector _center, float _radius, FRotator _centerRotation, float _startAngle, float _endAngle,float _sign) :isPoint(false) ,center(_center), radius(_radius), centerRotation(_centerRotation), startAngle(_startAngle), endAngle(_endAngle),sign(_sign) {
	};

	FCircle(FVector point,FRotator _centerRotation,float _startAngle,float _endAngle,float _sign) :isPoint(true) ,center(point), radius(0), centerCircle(point), startCircle(point), endCircle(point),centerRotation(_centerRotation),startAngle(_startAngle),endAngle(_endAngle),sign(_sign) {
	};

	FCircle():isPoint(true){
	};

	UPROPERTY()
		bool isPoint = false;

	UPROPERTY(BlueprintReadOnly)
		FVector center = FVector();

	UPROPERTY()
		float radius = 0;

	UPROPERTY(BlueprintReadOnly)
		FVector centerCircle = FVector();

	UPROPERTY(BlueprintReadOnly)
		FVector startCircle = FVector();

	UPROPERTY(BlueprintReadOnly)
		FVector endCircle = FVector();

	UPROPERTY()
		FRotator centerRotation = FRotator();

	UPROPERTY()
		float startAngle = 0;

	UPROPERTY()
		float endAngle = 0;
	
	UPROPERTY()
		float sign = 0;
};

USTRUCT(BlueprintType)
struct FClothoidPath
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FClothoidPath")
		FVector p_controll_Loc = FVector();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FClothoidPath")
		float phi1 = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FClothoidPath")
		float phi0 = 0;

};


UCLASS()
class TESTCLOTHOID_API AclothoidLine : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AclothoidLine();

	UPROPERTY(EditAnywhere,BlueprintReadOnly)
		TArray<UBillboardComponent*> billboards;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		TArray<float>phiValues;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		UBillboardComponent* pointMark;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		float speed = 1000;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Meta = (ClampMin = 0, ClampMax = 90))
		float turn_rate = 40;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Meta = (ClampMin = 0, ClampMax = 89))
		float turningPerformance = 80;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Meta = (ClampMin = 0, ClampMax = 250))
		float scale_calcTurnRadius = 250;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		float currentAngle = 0;

	UPROPERTY(EditAnywhere,BlueprintReadOnly)
		float currentSpeed = 0.0f;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		TArray<FVector2D>locs;

	UPROPERTY(BlueprintReadWrite)
		float test_phiZeroValue;

	UPROPERTY(BlueprintReadWrite)
		float test_phiOneValue;

	UPROPERTY()
		FCircle pre_Circle;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Meta = (ClampMin = 1, ClampMax = 100))
		int accuracy = 3;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Meta = (ClampMin = 1, ClampMax = 100))
		int arc_accuracy = 50;

	UPROPERTY(BlueprintReadOnly)
		int check_loopCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		float test_turnRate = 1;

	UPROPERTY(EditAnywhere)
		int check_CircleIndex = 0;

	UPROPERTY(BlueprintReadOnly)
		FCircle test_Circle = FCircle();

	UPROPERTY(BlueprintReadOnly)
		FVector test_location;

	/*
	UPROPERTY(BlueprintReadOnly)
		TArray<FVector>points;

	
	UPROPERTY(EditAnywhere)
		int n = 1000;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
		float _hValue = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float _phi0Value = 0.0f;

	UPROPERTY(EditAnywhere)
		float _phiVValue = 0.0f;

	UPROPERTY(EditAnywhere)
		float _phiUValue = 0.0f;
	*/

public:

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	/*
	UFUNCTION(BlueprintCallable)
		void CalcClothoidSplineResults();
	*/

	//double fx(int n, float phi1, float phi0, double x,float fov);

	double fx(int n, float phi,double x, float fov);

	double fx_d(int n,FDouble_ClothoidParameter &param, float fov);

	//(ノット,旋回性能(バンク角の代わりに使用))
	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		float calcTurnRadius(float turn_speed,float turnningPerformance,float scale);

	double angle_diff(double theta1,double theta2);

	float angle_diff2(float theta1,float theta2);

	FRotator delta_VectorAngle(FVector p1,FVector p2);

	void calcCircleInfo(FCircle &circle,FVector targetLocation);

	void test_circleCheck(FCircle &circle);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		float get_clothoid_angle(float radius);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		float calc_curve_angle(float length,float radius);

	UFUNCTION(BlueprintCallable,Category = CalcClothoidSpline)
		FVector calcMaxCircleAngle(FCircle &circle,FCircle &circle2);

	//float myNewton(float n, float _phi, float fov);

	float getPhi_newtonMethod(float n,float _phi,float fov);

	bool newtonMethod(const float localPhi1,const float localPhi0,float &phiV,float &phiU,float r,float theta);

	//二分法(単一クロソイド曲線用)
	bool calcNonlinearEquation(const float localPhi1, const float localPhi0, float& phiV, float& phiU, float r, float theta);

	//二分法(単一クロソイド曲線用(Ver2))
	//bool calcNonlinearEquationVer2(const float phi0,const float phiV, float& phiU,const float fov);

	//二分法(二連クロソイド曲線用)
	bool calcNonlinearEquationVer2(FDouble_ClothoidParameter&_param, const float fov);

	//二分法(３連クロソイド曲線用)
	bool calcNonlinearEquation_threeC(const float phi0, float& phiV, float& phiU, float r, float theta);

	//float falsePositionMethod(float n, float _phi, float fov);

	//FVector calcCurveHandle(FVector p1,FVector p2,float radius);

	UFUNCTION(BlueprintCallable)
		float getThreePointAngle(FVector p1,FVector p2,FVector p3);

	float getMiddleAngle(float angle1,float angle2);

	float getMiddleAngle_gimbal(float angle1,float angle2,bool sign);

	//FVector calcStartLocationOnArc(FVector current,FVector next,float arc_radius,FVector center,float sign);

	FVector calcStartLocationOnArc(FVector current,float arc_radius, FVector center, float sign);

	float convert_Angle_to_controllAngle(float angle);

	float calculateArcLength(float radius,float startAngle,float endAngle);

	TArray<FVector>calcCircleLocations(float startAngle,float endAngle,float radius,FVector curve_start,FVector curve_end,float sign);

	TArray<FVector>calcCircleLocations2(float startAngle, float endAngle, float radius, FVector curve_center,FVector curve_start, FVector curve_end, float sign);


	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		FVector getAngleLocation(float angle,float radius,FVector centerLocation);

	FVector getAngleLocationFromThreepoint(float thetaMax,FVector p1,FVector p2,FVector p3,float radius);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		TArray<FVector>calcClothoidSpline(UPARAM(ref) TArray<FClothoidPath>& pathDatas);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		TArray<FVector>calcDebugClothoidSpline(UPARAM(ref) TArray<FClothoidPath>& pathDatas);

	UFUNCTION()
		float combineFloat(float yaw1,float yaw2);

	UFUNCTION()
		float deltaFloat(float yaw1, float yaw2);

	//UFUNCTION(BlueprintCallable,Category = CalcClothoidSplineResultsSecond)
	//	TArray<FVector>CalcClothoidSplineResultsSecond(UPARAM(ref) TArray<FVector>& pathDatas);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidCurve)
		TArray<FVector>calcClothoidCurve(float phi1,float phi0,float length,float fov,FVector controllLoc,FVector curveStartLoc);

	UFUNCTION(BlueprintCallable,Category = clothoidCurve)
		TArray<FVector>clothoidCurve(int num,float h,float phi0,float phiV,float phiU,FVector controllLoc);

	float angleOf2DVector(FVector2D p1,FVector2D p2);

	float calc_TurnRate(FVector target,FVector p1,FVector p2);

	float calc_TurnRate2(FVector current, FVector center, FVector target);

	UFUNCTION(BlueprintCallable)
		float calc_TurnRate_Newton(float _sign, FCircle &circle, FVector _target);
		//float calc_TurnRate_Newton(float _sign,float _centerAngle,float _radius,FVector _center, FVector _target);


	UFUNCTION(BlueprintCallable)
		float calc_TurnRate_Newton2(FCircle &circle,FCircle &circle2);

	UFUNCTION(BlueprintCallable)
		float f_TurnRate(float value,float _sign, float _centerAngle, float _radius, FVector _center, FVector _target);

	//template <class T = FSlope, class Real = float, class R = float>
	void simpson_integral(FSlope f, float a, float b, std::complex<float> *r);

	void phiSimpson_integral(FPhiSlope f, float a, float b, std::complex<double>* r);

	FRotator getRotFromCircleTangent(FVector center,FVector circleTangent,int sign);

	//↓testFunction///
	UFUNCTION(BlueprintCallable)
		TArray<FVector>tripleClothoidCurve(FTriple_ClothoidParameter bp_Param,const float phi1,const float phi0,FVector start,FVector goal);

	//曲率変化率(phiU)のみ独立させて計算
	//初期角度(phi0)、初期曲率(phiV)は前回の制御点の数値参照
	//二連クロソイド曲線
	UFUNCTION(BlueprintCallable)
		TArray<FVector>multipleClothoidCurve(FDouble_ClothoidParameter bp_Param, FVector start, FVector goal);

	UFUNCTION()
		bool calcClothoidParameter(const float phi1,const float phi0,const float theta,const float r,float &h,FSlope &slope);

	UFUNCTION()
		bool calcClothoidParameterVer2(FDouble_ClothoidParameter &param, const float fov,const float pointDistance);

	UFUNCTION()
		bool calcTripleCLothoidParameter(FTriple_ClothoidParameter& param,float fov,float startAngle, float lastAngle, float a12, float a22, float a32,float curveture0,float curveture1,float h,float s1,float s2);

	UFUNCTION()
		bool calcTripleClothoidCurvePoint(FTriple_ClothoidParameter &param);
	
	float simpson_integral_cos(FSlope f, float a, float b);

	float simpson_integral_sin(FSlope f, float a, float b);

	void simpson_integral_phi1(FTriple_ClothoidParameter &param,float a,float b, std::complex<float>* r);

	void simpson_integral_phi2(FTriple_ClothoidParameter& param, float a, float b, std::complex<float>* r);

	void simpson_integral_phi3(FTriple_ClothoidParameter& param, float a, float b, std::complex<float>* r);

	UFUNCTION(BlueprintCallable)
		void test_Function();

		float angle_Vector(float value,float r);

		const float fxVer2(const FPhiSlope f,const float fov);

		const float fx_sin(FSlope &f);

		const float fx_cos(const float phi0, const float phi, const float x);

private:
	bool clothoidInitialize = false;

	bool preControllP = false;
};