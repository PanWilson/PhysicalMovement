// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BlueprintEditorModule.h"
#include "GameFramework/MovementComponent.h"
#include "PhysicalMovementComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FApexReachedSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FGravityChangedSignature, float, Gravity);


UCLASS(ClassGroup = Movement, meta = (BlueprintSpawnableComponent))
class PHYSICALMOVEMENT_API UPhysicalMovementComponent : public UMovementComponent
{
	GENERATED_BODY()

protected:
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	float MaxSpeed;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	float MaxAccelForce;
	
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	FRuntimeFloatCurve MaxAccelerationForceFactorFromDot;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	FVector ForceScale;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	FVector StartTraceOffset;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	FVector TraceDirection;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	float TraceLength;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	float FloatHeight;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	float FloatSpringStrength;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	float FloatSpringDamping;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	TEnumAsByte<ECollisionChannel> FloatTraceChannel;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Floating")
	bool bFloorSnappingEnabled;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Orientation")
	float OrientationSpringStrength;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Orientation")
	float OrientationSpringDamping;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float JumpHeight;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float JumpDistance;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	bool bGravityEnabled;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float BaseGravity;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float JumpSwitchTime;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float InitalVerticalVelocity;
	
	FTimerHandle JumpTimer;
	
	UPROPERTY(Transient)
	float JumpGravity;
	
	UPROPERTY(BlueprintReadOnly, Transient)
	float GravityScale;

	UPROPERTY(Transient)
	float InitialJumpVelocity;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	FVector GravityDirection;

	bool bWantsToJump;

	UPROPERTY(Transient)
	float JumpRequestTime;
	
	UPROPERTY(Transient)
	float LastOnTheGroundTime;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float JumpBufferTime;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float CoyoteTime;
	
	UPROPERTY(Transient)
	UPrimitiveComponent* OwnerPrimitiveCompo;

	UPROPERTY(Transient, DuplicateTransient)
	TObjectPtr<class APawn> PawnOwner;

	UPROPERTY(Transient)
	FVector GroundVelocity;

	UPROPERTY(Transient)
	FQuat PawnOrientation;

	UPROPERTY(Transient, BlueprintReadWrite, Category = Movement)
	bool bIsOnTheGround;

	UPROPERTY(Transient)
	bool bRequestApexCheck;

	UPROPERTY(Transient)
	bool bCheckForApex;
	
	bool bFloatingEnabled;

	UPROPERTY(Transient)
	FVector OldVelocity;
	
public:

	//Engine overrides

	UPhysicalMovementComponent();
	
	virtual void BeginPlay() override;
	
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

	virtual void SetUpdatedComponent(USceneComponent* NewUpdatedComponent) override;

	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void AddInputVector(FVector WorldVector, bool bForce = false);

	UFUNCTION(BlueprintCallable, Category="Movement", meta=(Keywords="GetInput"))
	FVector GetPendingInputVector() const;

	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void Jump();
	
	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void StopJump();

	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual FVector ConsumeInputVector();

	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void SetGravity(const float InGravity);
	
	//Events
	
	UPROPERTY(BlueprintAssignable);
	FApexReachedSignature OnApexReached;

	UPROPERTY(BlueprintAssignable);
	FGravityChangedSignature OnGravityChanged;
	
	//Utils

	static FQuat GetShortestRotation(FQuat CurrentOrientation, FQuat TargetOrientation);
protected:
	
	//Physical movement
	void ComputeAndApplyFloatingSpring();

	void ComputeAndApplyOrientationSpring() const;

	void ApplyInputForces(const float DeltaTime);

	void ApplyGravity() const;

	void FallingAfterJumpCheck();

	void OnStoppedJumping();

	void CheckIfWantsToJump();
};
