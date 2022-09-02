// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BlueprintEditorModule.h"
#include "GameFramework/PawnMovementComponent.h"
#include "PhysicalMovementComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FApexReachedSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FGravityChangedSignature, float, Gravity);


UCLASS(ClassGroup = Movement, meta = (BlueprintSpawnableComponent))
class PHYSICALMOVEMENT_API UPhysicalMovementComponent : public UPawnMovementComponent
{
	GENERATED_BODY()

protected:
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	float MaxSpeed;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	float MaxAccelForce;
	
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	FRuntimeFloatCurve MaxDecelerationAtSpeed;
	
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
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Orientation")
	bool bRotateTowardsMovement;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Orientation")
	bool bUseControlRotation;
	
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
	float InitialVerticalVelocity;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	FVector GravityDirection;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float MinJumpTime;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float JumpBufferTime;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | Jump")
	float CoyoteTime;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsMovement | AI")
	float BrakingDecelerationWalking;
	
	FTimerHandle FallTimerHandle;
	
	UPROPERTY(Transient)
	float JumpGravity;
	
	UPROPERTY(BlueprintReadOnly, Transient)
	float GravityScale;

	UPROPERTY(Transient)
	float InitialJumpVelocity;
	
	bool bWantsToJump;

	UPROPERTY(Transient)
	float JumpRequestTime;
	
	UPROPERTY(Transient)
	float LastOnTheGroundTime;
	
	UPROPERTY(Transient)
	UPrimitiveComponent* UpdatePrimitive;
	
	UPROPERTY(Transient)
	FVector CharacterVelocity;

	UPROPERTY(Transient)
	float CharacterSpeed;
	
	UPROPERTY(Transient)
	FVector StandVelocity;

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
	
	UPROPERTY(Transient)
	FVector RequestedVelocity;
	
	UPROPERTY(Transient)
	bool bHasRequestedVelocity;
	
	UPROPERTY(BlueprintReadWrite, Category = "PhysicsMovement | Movement")
	bool bInputEnabled;
	
	bool bHasMoveToRequest;

	float RemainingMoveToTime;
	
	FVector MoveToVelocity;

	UPROPERTY(Transient)
	float CurrentMaxAccelForce;
	
public:

	//Engine overrides

	UPhysicalMovementComponent();
	
	virtual void BeginPlay() override;
	
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

	virtual void SetUpdatedComponent(USceneComponent* NewUpdatedComponent) override;

	//Interface

	//BEGIN UMovementComponent Interface
	virtual float GetMaxSpeed() const override;
	virtual void StopActiveMovement() override;
	virtual bool IsCrouching() const override;
	virtual bool IsFalling() const override;
	virtual bool IsMovingOnGround() const override;
	virtual bool IsSwimming() const override;
	virtual bool IsFlying() const override;
	virtual float GetGravityZ() const override;
	virtual void AddRadialForce(const FVector& Origin, float Radius, float Strength, enum ERadialImpulseFalloff Falloff) override;
	virtual void AddRadialImpulse(const FVector& Origin, float Radius, float Strength, enum ERadialImpulseFalloff Falloff, bool bVelChange) override;
	//END UMovementComponent Interface
	
	//BEGIN UNavMovementComponent Interface
	virtual void RequestDirectMove(const FVector& MoveVelocity, bool bForceMaxSpeed) override;
	virtual void RequestPathMove(const FVector& MoveInput) override;
	virtual bool CanStartPathFollowing() const override;
	virtual bool CanStopPathFollowing() const override;
	virtual float GetPathFollowingBrakingDistance(float InMaxSpeed) const override;
	//END UNaVMovementComponent Interface

	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void Jump();
	
	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void StopJump();

	UFUNCTION(BlueprintCallable, Category="Movement")
	virtual void SetGravity(const float InGravity);
	
	UFUNCTION(BlueprintCallable, Category="Movement")
	float GetWorldGravityZ() const ;
	
	UFUNCTION(BlueprintCallable, BlueprintPure, Category="Movement")
	virtual FVector GetStandVelocity();
	
	UFUNCTION(BlueprintCallable, BlueprintPure, Category="Movement")
	virtual FVector GetRelativeVelocity();

	UFUNCTION(BlueprintCallable, Category="Movement")
	void EnableInput(bool bInEnable = true);

	UFUNCTION(BlueprintCallable, Category="Movement")
	void MoveTo(FVector InLocation, float Time);

	UFUNCTION(BlueprintCallable, Category="Movement")
	void StopMoveTo();
	
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

	void ComputeAndApplyOrientationSpring();

	void ApplyInputForces(const float DeltaTime);

	void ApplyGravity() const;
	
	void OnStoppedJumping();

	void CheckIfWantsToJump();
	
	void FallingAfterJumpCheck();
	
	void ReachedApex();

	void UpdateMoveTo(float DeltaTime);

};
