// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <complex>
#include <functional>
#include <iterator>
#include <numeric>
#include "clothoidLine.generated.h"

class UBillboardComponent;

USTRUCT()
struct FSlope {
	GENERATED_USTRUCT_BODY()

	float phi0;
	float phiV;
	float phiU;

	//@0+@1s+@2ss
	float phi(float _phi0, float _phiV, float _phiU, float S) {
		return _phi0 + _phiV * S + _phiU * (S * S);
	}

	std::complex<float> slope_f(float _phi0, float _phiV, float _phiU, float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * phi(_phi0, _phiV, _phiU, S));
	}

	std::complex<float> operator()(float S) {
		return slope_f(phi0, phiV, phiU, S);
	}
};

USTRUCT()
struct FPhiSlope {
	GENERATED_USTRUCT_BODY()

	float phiV;
	float phiU;

	std::complex<float> slope_f(float _phiV, float _phiU, float S) {
		std::complex<float> j(0.0f, 1.0f);
		return std::exp(j * (phiV * S + phiU * (S * S)));
	}

	std::complex<float> operator()(float S) {
		return slope_f(phiV, phiU, S);
	}
};

USTRUCT(BlueprintType)
struct FClothoidPath
{
	GENERATED_BODY()

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

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		TArray<FVector2D>locs;

	UPROPERTY(BlueprintReadWrite)
		float test_phiZeroValue;

	UPROPERTY(BlueprintReadWrite)
		float test_phiOneValue;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Meta = (ClampMin = 1, ClampMax = 100))
		int accuracy = 50;

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

	//(ノット,旋回性能(バンク角の代わりに使用))
	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		float calcTurnRadius(float turn_speed,float turnningPerformance,float scale);

	double angle_diff(double theta1,double theta2);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		float get_clothoid_angle(float radius);

	float myNewton(float n, float _phi, float fov);

	float getPhi_newtonMethod(float n,float _phi,float fov);

	float falsePositionMethod(float n, float _phi, float fov);

	FVector calcCurveHandle(FVector p1,FVector p2,float radius);

	float getThreePointAngle(FVector p1,FVector p2,FVector p3);

	float getMiddleAngle(float angle1,float angle2);

	float convert_Angle_to_controllAngle(float angle);

	FVector getAngleLocation(float angle,float radius,FVector centerLocation);

	FVector getAngleLocationFromThreepoint(float thetaMax,FVector p1,FVector p2,FVector p3,float radius);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidSpline)
		TArray<FVector>calcClothoidSpline(UPARAM(ref) TArray<FClothoidPath>& pathDatas);

	//UFUNCTION(BlueprintCallable,Category = CalcClothoidSplineResultsSecond)
	//	TArray<FVector>CalcClothoidSplineResultsSecond(UPARAM(ref) TArray<FVector>& pathDatas);

	UFUNCTION(BlueprintCallable, Category = CalcClothoidCurve)
		TArray<FVector2D>calcClothoidCurve(int n, float phi1,float phi0,float straightDis,float fov);

	float angleOf2DVector(FVector2D p1,FVector2D p2);

	//template <class T = FSlope, class Real = float, class R = float>
	void simpson_integral(FSlope f, float a, float b, std::complex<float> *r);

	void phiSimpson_integral(FPhiSlope f, float a, float b, std::complex<float>* r);

	
private:

};