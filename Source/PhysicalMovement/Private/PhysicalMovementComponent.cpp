// Fill out your copyright notice in the Description page of Project Settings.


#include "PhysicalMovementComponent.h"

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
}

void UPhysicalMovementComponent::BeginPlay()
{
	Super::BeginPlay();
	
	OwnerPrimitiveCompo = Cast<UPrimitiveComponent>(GetOwner()->GetComponentByClass(UPrimitiveComponent::StaticClass()));
	if (OwnerPrimitiveCompo != nullptr)
	{
		OwnerPrimitiveCompo->SetEnableGravity(false);
	}

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

	if (OwnerPrimitiveCompo != nullptr && OwnerPrimitiveCompo->IsSimulatingPhysics())
	{
		CharacterVelocity = OwnerPrimitiveCompo->GetComponentVelocity();
		ComputeAndApplyFloatingSpring();
		ApplyInputForces(DeltaTime);
		ComputeAndApplyOrientationSpring();
		ApplyGravity();
		FallingAfterJumpCheck();
		OldVelocity = OwnerPrimitiveCompo->GetComponentVelocity();
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

void UPhysicalMovementComponent::AddInputVector(FVector WorldVector, bool bForce)
{
	if (PawnOwner)
	{
		PawnOwner->Internal_AddMovementInput(WorldVector, bForce);
	}
}

FVector UPhysicalMovementComponent::GetPendingInputVector() const
{
	return PawnOwner ? PawnOwner->Internal_GetPendingMovementInputVector() : FVector::ZeroVector;
}

void UPhysicalMovementComponent::Jump()
{
	if (bIsOnTheGround || (LastOnTheGroundTime + CoyoteTime >= GetWorld()->GetWorld()->GetTimeSeconds()))
	{
		bWantsToJump = false;
		bFloatingEnabled = false;
		SetGravity(JumpGravity);
		OwnerPrimitiveCompo->AddImpulse(TraceDirection * -(InitialVerticalVelocity - FMath::Min(OwnerPrimitiveCompo->GetComponentVelocity().Z,0.f)), NAME_None, true);
		GetWorld()->GetTimerManager().SetTimer(FallTimerHandle, this, &UPhysicalMovementComponent::OnStoppedJumping,JumpSwitchTime);
		bRequestApexCheck = true;
	}
	else
	{
		JumpRequestTime = GetWorld()->GetTimeSeconds();
		bWantsToJump = true;
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

FVector UPhysicalMovementComponent::ConsumeInputVector()
{
	return PawnOwner ? PawnOwner->Internal_ConsumeMovementInputVector() : FVector::ZeroVector;
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
		const FVector OwnerVelocity = OwnerPrimitiveCompo->GetComponentVelocity();

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
			OwnerPrimitiveCompo->AddForce(TraceDirection * SpringForce * OwnerPrimitiveCompo->GetMass());
			
			if (OtherComponent != nullptr)
			{
				StandVelocity = OtherComponent->GetComponentVelocity();

				if (OtherComponent->IsSimulatingPhysics())
				{
					OtherComponent->AddForceAtLocation(TraceDirection * -SpringForce * OwnerPrimitiveCompo->GetMass(),
													   OutHit.Location);
				}
			}
		}
	}

}

void UPhysicalMovementComponent::ComputeAndApplyOrientationSpring() const
{
	const FQuat OwnerOrientation = OwnerPrimitiveCompo->GetComponentTransform().GetRotation();
	const FQuat ToGoal = GetShortestRotation(OwnerOrientation, PawnOrientation);

	FVector OrientationAxis;
	float OrientationAngle;
	
	ToGoal.ToAxisAndAngle(OrientationAxis, OrientationAngle);

	const FVector TorqueForce = (OrientationAxis * (OrientationAngle * OrientationSpringStrength)) - (OwnerPrimitiveCompo->GetPhysicsAngularVelocityInRadians() * OrientationSpringDamping);

	OwnerPrimitiveCompo->AddTorqueInRadians(TorqueForce * OwnerPrimitiveCompo->GetMass());
}

void UPhysicalMovementComponent::ApplyInputForces(const float DeltaTime)
{
	if (const AController* Controller = PawnOwner->GetController(); Controller && Controller->IsLocalController())
	{
		const FVector ControlDirection = GetPendingInputVector().GetClampedToMaxSize(1.f);
		
		if(!FMath::IsNearlyZero(ControlDirection.SizeSquared()))
		{
			PawnOrientation = ControlDirection.ToOrientationQuat();
		}
		const float VelocityDot = ControlDirection.GetSafeNormal() | CharacterVelocity;
		
		const float FinalAcceleration = (MaxAccelForce * MaxAccelerationForceFactorFromDot.GetRichCurveConst()->Eval(VelocityDot));
		
		const FVector GoalVelocity = ControlDirection * MaxSpeed;

		FVector NeededAcceleration = ((GoalVelocity + (StandVelocity * ForceScale)) - (CharacterVelocity * ForceScale)) / DeltaTime;
		NeededAcceleration = NeededAcceleration.GetClampedToMaxSize(FinalAcceleration);
		
		OwnerPrimitiveCompo->AddForce(NeededAcceleration * OwnerPrimitiveCompo->GetMass() * ForceScale);
		
		ConsumeInputVector();
	}
}

void UPhysicalMovementComponent::ApplyGravity() const
{
	if (bGravityEnabled)
	{
		OwnerPrimitiveCompo->AddForce(GravityDirection * -GravityScale * GetGravityZ(), NAME_None, true);
	}
}

void UPhysicalMovementComponent::FallingAfterJumpCheck()
{
	if (bCheckForApex && OwnerPrimitiveCompo->GetComponentVelocity().Z < 0.f)
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
	GravityScale = InGravity / GetGravityZ();
	OnGravityChanged.Broadcast(GravityScale * GetGravityZ());
}

FVector UPhysicalMovementComponent::GetStandVelocity()
{
	return StandVelocity;
}

FVector UPhysicalMovementComponent::GetRelativeVelocity()
{
	return CharacterVelocity - StandVelocity;
}



