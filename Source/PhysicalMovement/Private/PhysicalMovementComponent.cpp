// Fill out your copyright notice in the Description page of Project Settings.


#include "PhysicalMovementComponent.h"

#include "AITestsCommon.h"
#include "GameFramework/PhysicsVolume.h"
#include "PhysicsEngine/PhysicsSettings.h"

UPhysicalMovementComponent::UPhysicalMovementComponent()
{
	//Movement
	MaxSpeed = 500.f;
	MaxAccelForce = 1000.f;
	ForceScale = FVector(1.f, 1.f, 0.f);

	//Floating
	StartTraceOffset = FVector::ZeroVector;
	TraceDirection = FVector(0.f, 0.f, -1.f);
	TraceLength = 85.f;
	FloatHeight = 80.f;
	FloatSpringStrength	= 100.f;
	FloatSpringDamping = .3f;
	FloatTraceChannel = ECollisionChannel::ECC_Visibility;
	bFloorSnappingEnabled = true;

	//Orientation
	OrientationSpringStrength = 9375.f;
	OrientationSpringDamping = 2812.f;
	bRotateTowardsMovement = true;
	bUseControlRotation = false;
	
	//Jump
	JumpHeight = 120.f;
	JumpDistance = 400.f;
	bGravityEnabled = true;
	BaseGravity = -1470.f;
	JumpSwitchTime = .3f;
	InitialVerticalVelocity = 600.f;
	GravityDirection = FVector(0.f, 0.f, -1.f);
	MinJumpTime = .1f;
	JumpBufferTime = .1f;
	CoyoteTime = .1f;
	bFloatingEnabled = true;
	bWantsToJump = false;
	bInputEnabled = true;
	bHasMoveToRequest = false;
}

void UPhysicalMovementComponent::BeginPlay()
{
	Super::BeginPlay();
	
	UpdatePrimitive = Cast<UPrimitiveComponent>(GetOwner()->GetComponentByClass(UPrimitiveComponent::StaticClass()));
	if (UpdatePrimitive != nullptr)
	{
		UpdatePrimitive->SetEnableGravity(false);
	}

	CurrentMaxAccelForce = MaxAccelForce;
	
	PawnOrientation = GetOwner()->GetActorForwardVector().ToOrientationQuat();
	
	//Initialization of jumping
	const float MaxFallTime = FMath::Sqrt((-2.f*JumpHeight)/BaseGravity);
	const float TotalTime = JumpDistance/MaxSpeed;
	const float NeededInitialVelocity = -1.f * BaseGravity * ((TotalTime/2.f)- JumpSwitchTime);
	JumpGravity = (NeededInitialVelocity - InitialVerticalVelocity)/JumpSwitchTime;
	
	SetGravity(BaseGravity);
}

void UPhysicalMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (UpdatePrimitive != nullptr && UpdatePrimitive->IsSimulatingPhysics())
	{
		CharacterVelocity = UpdatePrimitive->GetComponentVelocity();
		CharacterSpeed = CharacterVelocity.Size();
		ComputeAndApplyFloatingSpring();
		UpdateMoveTo(DeltaTime);
		ApplyInputForces(DeltaTime);
		ComputeAndApplyOrientationSpring();
		ApplyGravity();
		FallingAfterJumpCheck();
		OldVelocity = UpdatePrimitive->GetComponentVelocity();
	}
}

void UPhysicalMovementComponent::SetUpdatedComponent(USceneComponent* NewUpdatedComponent)
{
	if (NewUpdatedComponent)
	{
		if (!ensureMsgf(Cast<APawn>(NewUpdatedComponent->GetOwner()), TEXT("%s must update a component owned by a Pawn"), *GetName()))
		{
			return;
		}
	}

	Super::SetUpdatedComponent(NewUpdatedComponent);

	PawnOwner = UpdatedComponent ? CastChecked<APawn>(UpdatedComponent->GetOwner()) : nullptr;
}

float UPhysicalMovementComponent::GetMaxSpeed() const
{
	return MaxSpeed;
}

void UPhysicalMovementComponent::StopActiveMovement()
{
	Super::StopActiveMovement();
}

bool UPhysicalMovementComponent::IsCrouching() const
{
	return false;
}

bool UPhysicalMovementComponent::IsFalling() const
{
	return !bIsOnTheGround && !bCheckForApex;
}

bool UPhysicalMovementComponent::IsMovingOnGround() const
{
	return bIsOnTheGround;
}

bool UPhysicalMovementComponent::IsSwimming() const
{
	return false;
}

bool UPhysicalMovementComponent::IsFlying() const
{
	return false;
}

float UPhysicalMovementComponent::GetGravityZ() const
{
	return BaseGravity * GravityScale;
}

void UPhysicalMovementComponent::AddRadialForce(const FVector& Origin, float Radius, float Strength,
	ERadialImpulseFalloff Falloff)
{
	UpdatePrimitive->AddRadialForce(Origin, Radius, Strength, Falloff);
}

void UPhysicalMovementComponent::AddRadialImpulse(const FVector& Origin, float Radius, float Strength,
	ERadialImpulseFalloff Falloff, bool bVelChange)
{
	UpdatePrimitive->AddRadialImpulse(Origin, Radius, Strength, Falloff, bVelChange);
}

void UPhysicalMovementComponent::RequestDirectMove(const FVector& MoveVelocity, bool bForceMaxSpeed)
{
	if (MoveVelocity.SizeSquared() < KINDA_SMALL_NUMBER)
	{
		return;
	}

	// if (ShouldPerformAirControlForPathFollowing())
	// {
	// 	const FVector FallVelocity = MoveVelocity.GetClampedToMaxSize(GetMaxSpeed());
	// 	PerformAirControlForPathFollowing(FallVelocity, FallVelocity.Z);
	// 	return;
	// }

	RequestedVelocity = MoveVelocity;
	bHasRequestedVelocity = true;

	if (IsMovingOnGround())
	{
		RequestedVelocity.Z = 0.0f;
	}
}

void UPhysicalMovementComponent::RequestPathMove(const FVector& MoveInput)
{
	FVector AdjustedMoveInput(MoveInput);

	// preserve magnitude when moving on ground/falling and requested input has Z component
	// see ConstrainInputAcceleration for details
	if (MoveInput.Z != 0.f && (IsMovingOnGround() || IsFalling()))
	{
		const float Mag = MoveInput.Size();
		AdjustedMoveInput = MoveInput.GetSafeNormal2D() * Mag;
	}
	
	Super::RequestPathMove(AdjustedMoveInput);
}

bool UPhysicalMovementComponent::CanStartPathFollowing() const
{
	if (UpdatePrimitive != nullptr && !UpdatePrimitive->IsSimulatingPhysics())
	{
		return false;
	}

	return Super::CanStartPathFollowing();
}

bool UPhysicalMovementComponent::CanStopPathFollowing() const
{
	return !IsFalling();
}

float UPhysicalMovementComponent::GetPathFollowingBrakingDistance(float InMaxSpeed) const
{
	if (bUseFixedBrakingDistanceForPaths)
	{
		return FixedPathBrakingDistance;
	}

	const float BrakingDeceleration = FMath::Abs(BrakingDecelerationWalking);

	// character won't be able to stop with negative or nearly zero deceleration, use MaxSpeed for path length calculations
	const float BrakingDistance = (BrakingDeceleration < SMALL_NUMBER) ? MaxSpeed : (FMath::Square(MaxSpeed) / (2.f * BrakingDeceleration));
	return BrakingDistance;
}

void UPhysicalMovementComponent::Jump()
{
	if (bInputEnabled)
	{
		if (bIsOnTheGround || (LastOnTheGroundTime + CoyoteTime >= GetWorld()->GetWorld()->GetTimeSeconds()))
		{
			bWantsToJump = false;
			bFloatingEnabled = false;
			SetGravity(JumpGravity);
			UpdatePrimitive->AddImpulse(TraceDirection * -(InitialVerticalVelocity - FMath::Min(UpdatePrimitive->GetComponentVelocity().Z,0.f)), NAME_None, true);
			GetWorld()->GetTimerManager().SetTimer(FallTimerHandle, this, &UPhysicalMovementComponent::OnStoppedJumping,JumpSwitchTime);
			bRequestApexCheck = true;
		}
		else
		{
			JumpRequestTime = GetWorld()->GetTimeSeconds();
			bWantsToJump = true;
		}
	}
}

void UPhysicalMovementComponent::StopJump()
{
	if (FallTimerHandle.IsValid())
	{
		const float TimeSinceJump = GetWorld()->GetTimerManager().GetTimerElapsed(FallTimerHandle);
		if (TimeSinceJump >= MinJumpTime)
		{
			OnStoppedJumping();
		}
		else
		{
			GetWorld()->GetTimerManager().ClearTimer(FallTimerHandle);
			GetWorld()->GetTimerManager().SetTimer(FallTimerHandle, this, &UPhysicalMovementComponent::OnStoppedJumping, MinJumpTime - TimeSinceJump);
		}
	}
}

FQuat UPhysicalMovementComponent::GetShortestRotation(FQuat CurrentOrientation, FQuat TargetOrientation)
{
	if ((TargetOrientation | CurrentOrientation) < 0)
	{
		return TargetOrientation * (CurrentOrientation * -1.f).Inverse();
	}
	else
	{
		return TargetOrientation * CurrentOrientation.Inverse();
	}

}

void UPhysicalMovementComponent::ComputeAndApplyFloatingSpring()
{
	const FVector OwnerLocation = GetOwner()->GetActorLocation();

	FHitResult OutHit;

	const FVector TraceStart = OwnerLocation + StartTraceOffset;
	const FVector TraceEnd = TraceStart + (TraceDirection * TraceLength);

	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(GetOwner());

	GetWorld()->LineTraceSingleByChannel(OutHit, TraceStart, TraceEnd, FloatTraceChannel,
	                                     CollisionParams);
	bool bOldIsOnTheGround = bIsOnTheGround;
	bIsOnTheGround = OutHit.bBlockingHit;
	
	if (!bOldIsOnTheGround && bIsOnTheGround)
	{
		if ((JumpRequestTime + JumpBufferTime) > GetWorld()->GetTimeSeconds())
		{
			CheckIfWantsToJump();
		}
	}
	
	if (OutHit.bBlockingHit)
	{
		LastOnTheGroundTime = GetWorld()->GetTimeSeconds();
		const FVector OwnerVelocity = UpdatePrimitive->GetComponentVelocity();

		UPrimitiveComponent* OtherComponent = OutHit.GetComponent();
		if (OtherComponent != nullptr)
		{
			StandVelocity = OtherComponent->GetComponentVelocity();
		}
		else
		{
			StandVelocity = FVector::ZeroVector;
		}

		const float DotDirectionVelocity = OwnerVelocity | TraceDirection;
		const float DotOtherDirectionVelocity = StandVelocity | TraceDirection;

		float RelativeVelocity = DotDirectionVelocity - DotOtherDirectionVelocity;

		float SpringDisplacement = OutHit.Distance  - FloatHeight;

		float SpringForce = (SpringDisplacement * FloatSpringStrength) - ((RelativeVelocity/GetWorld()->GetDeltaSeconds()) * FloatSpringDamping);
		SpringForce = bFloorSnappingEnabled ? SpringForce : FMath::Min(0.f, SpringForce);

		if (bFloatingEnabled)
		{
			UpdatePrimitive->AddForce(TraceDirection * SpringForce * UpdatePrimitive->GetMass());
			
			if (OtherComponent != nullptr)
			{
				StandVelocity = OtherComponent->GetComponentVelocity();

				if (OtherComponent->IsSimulatingPhysics())
				{
					OtherComponent->AddForceAtLocation(TraceDirection * -SpringForce * UpdatePrimitive->GetMass(),
													   OutHit.Location);
				}
			}
		}
	}

}

void UPhysicalMovementComponent::ComputeAndApplyOrientationSpring()
{
	if (bUseControlRotation && PawnOwner->GetController())
	{
		PawnOrientation = PawnOwner->GetController()->GetDesiredRotation().Quaternion();
	}
	const FQuat OwnerOrientation = UpdatePrimitive->GetComponentTransform().GetRotation();
	const FQuat ToGoal = GetShortestRotation(OwnerOrientation, PawnOrientation);

	FVector OrientationAxis;
	float OrientationAngle;
	
	ToGoal.ToAxisAndAngle(OrientationAxis, OrientationAngle);

	const FVector TorqueForce = (OrientationAxis * (OrientationAngle * OrientationSpringStrength)) - (UpdatePrimitive->GetPhysicsAngularVelocityInRadians() * OrientationSpringDamping);

	UpdatePrimitive->AddTorqueInRadians(TorqueForce * UpdatePrimitive->GetMass());
}

void UPhysicalMovementComponent::ApplyInputForces(const float DeltaTime)
{
	if (const AController* Controller = PawnOwner->GetController(); Controller && Controller->IsLocalController())
	{
		FVector ControlDirection = GetPendingInputVector().GetClampedToMaxSize(1.f);
		
		if (!bInputEnabled)
		{
			ControlDirection = FVector::ZeroVector;
		}
		
		if (bRotateTowardsMovement && !FMath::IsNearlyZero(ControlDirection.SizeSquared()))
		{
			PawnOrientation = ControlDirection.ToOrientationQuat();
		}
		const float VelocityDot = ControlDirection.GetSafeNormal() | CharacterVelocity;

		FVector GoalVelocity = ControlDirection * MaxSpeed;

		if (bHasRequestedVelocity)
		{
			GoalVelocity = RequestedVelocity;
			bHasRequestedVelocity = false;
			RequestedVelocity = FVector::ZeroVector;
		}

		const FVector NetVelocity = GoalVelocity + (StandVelocity * ForceScale);
		FVector NeededAcceleration = (NetVelocity - (CharacterVelocity * ForceScale))/ DeltaTime;
		
		const bool bIsDecelerating = (NeededAcceleration.SizeSquared() > SMALL_NUMBER) && (FMath::IsNearlyZero(CharacterVelocity.SizeSquared()) || ((CharacterVelocity | NeededAcceleration) < 0.f));
		
		float AccelerationLimit = 0.f;
		if (!bIsDecelerating)
		{
			AccelerationLimit = (CurrentMaxAccelForce * MaxAccelerationForceFactorFromDot.GetRichCurveConst()->Eval(VelocityDot));
		}
		else
		{
			AccelerationLimit = MaxDecelerationAtSpeed.GetRichCurve()->Eval(CharacterSpeed);
		}
		
		NeededAcceleration = NeededAcceleration.GetClampedToMaxSize(AccelerationLimit);

		UpdatePrimitive->AddForce(NeededAcceleration * UpdatePrimitive->GetMass() * ForceScale);
		ConsumeInputVector();
	}
}

void UPhysicalMovementComponent::ApplyGravity() const
{
	if (bGravityEnabled)
	{
		UpdatePrimitive->AddForce(GravityDirection * -GravityScale * GetWorldGravityZ(), NAME_None, true);
	}
}

void UPhysicalMovementComponent::FallingAfterJumpCheck()
{
	if (bCheckForApex && UpdatePrimitive->GetComponentVelocity().Z < 0.f)
	{
		ReachedApex();
		bCheckForApex = false;
	}
	
	//I Have problem with order of apex check since it can happen right after jump and velocity can still be <0
	//so i wait one Tick to check. Maybe there is better solution
	if (bRequestApexCheck)
	{
		bCheckForApex = true;
		bRequestApexCheck = false;
	}
}

void UPhysicalMovementComponent::ReachedApex()
{
	OnApexReached.Broadcast();
}

void UPhysicalMovementComponent::UpdateMoveTo(float DeltaTime)
{
	if (bHasMoveToRequest)
	{
		RemainingMoveToTime -= DeltaTime;
		if (RemainingMoveToTime>0.f)
		{
			RequestedVelocity = MoveToVelocity;
			bHasRequestedVelocity = true;
		}
		else
		{
			StopMoveTo();
		}
	}
}

void UPhysicalMovementComponent::OnStoppedJumping()
{
	SetGravity(BaseGravity);
	bRequestApexCheck = false;
	bFloatingEnabled = true;
}

void UPhysicalMovementComponent::CheckIfWantsToJump()
{
	if (bWantsToJump)
	{
		Jump();
	}
}

void UPhysicalMovementComponent::SetGravity(const float InGravity)
{
	GravityScale = InGravity / GetWorldGravityZ();
	OnGravityChanged.Broadcast(GravityScale * GetWorldGravityZ());
}

float UPhysicalMovementComponent::GetWorldGravityZ() const
{
	APhysicsVolume* PhysicsVolume = GetPhysicsVolume();
	return PhysicsVolume ? PhysicsVolume->GetGravityZ() : UPhysicsSettings::Get()->DefaultGravityZ;
}

FVector UPhysicalMovementComponent::GetStandVelocity()
{
	return StandVelocity;
}

FVector UPhysicalMovementComponent::GetRelativeVelocity()
{
	return CharacterVelocity - StandVelocity;
}

void UPhysicalMovementComponent::EnableInput(bool bInEnable)
{
	bInputEnabled = bInEnable;
}

void UPhysicalMovementComponent::MoveTo(FVector InLocation, float Time)
{
	const FVector FromTo = InLocation - GetOwner()->GetActorLocation();
	MoveToVelocity = FromTo/Time;
	RemainingMoveToTime = Time;
	CurrentMaxAccelForce = 1000000.f;
	bHasMoveToRequest = true;
	UpdateMoveTo(0.f);
}

void UPhysicalMovementComponent::StopMoveTo()
{
	bHasMoveToRequest = false;
	CurrentMaxAccelForce = MaxAccelForce;
	MoveToVelocity = FVector::ZeroVector;
}



