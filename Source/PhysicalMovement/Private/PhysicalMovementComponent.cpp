// Fill out your copyright notice in the Description page of Project Settings.


#include "PhysicalMovementComponent.h"

UPhysicalMovementComponent::UPhysicalMovementComponent()
{
	StartTraceOffset = FVector::ZeroVector;
	TraceDirection = FVector(0.f, 0.f, -1.f);
	TraceLength = 100.f;
	FloatHeight = 20.f;
	FloatSpringStrength	= 25.f;
	FloatSpringDamping = 3.f;
	FloatTraceChannel = ECollisionChannel::ECC_Visibility;
	
	OrientationSpringStrength = 20.f;
	OrientationSpringDamping = 3.f;

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
	JumpGravity = (NeededInitialVelocity - InitalVerticalVelocity)/JumpSwitchTime;
	InitialJumpVelocity = InitalVerticalVelocity;

	//InitialJumpVelocity = (2.f * JumpHeight * MaxSpeed) / AscendTravelDistance;
	//JumpGravity = (-2.f * JumpHeight * FMath::Square(MaxSpeed)) / FMath::Square(AscendTravelDistance);
	SetGravity(BaseGravity);
}

void UPhysicalMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (OwnerPrimitiveCompo != nullptr && OwnerPrimitiveCompo->IsSimulatingPhysics())
	{
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
		OwnerPrimitiveCompo->AddImpulse(TraceDirection * -InitialJumpVelocity, NAME_None, true);
		GetWorld()->GetTimerManager().SetTimer(JumpTimer, this, &UPhysicalMovementComponent::StopJump,JumpSwitchTime);
		//bRequestApexCheck = true;
	}
	else
	{
		JumpRequestTime = GetWorld()->GetTimeSeconds();
		bWantsToJump = true;
	}
}

void UPhysicalMovementComponent::StopJump()
{
	//if (bRequestApexCheck)
	{
		OnStoppedJumping();
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
		FVector OtherVelocity = FVector::ZeroVector;
		if (OtherComponent != nullptr)
		{
			OtherVelocity = OtherComponent->GetComponentVelocity();
		}

		const float DotDirectionVelocity = OwnerVelocity | TraceDirection;
		const float DotOtherDirectionVelocity = OtherVelocity | TraceDirection;

		float RelativeVelocity = DotDirectionVelocity - DotOtherDirectionVelocity;

		float SpringDisplacement = OutHit.Distance  - FloatHeight;

		float SpringForce = (SpringDisplacement * FloatSpringStrength) - ((RelativeVelocity/GetWorld()->GetDeltaSeconds()) * FloatSpringDamping);
		SpringForce = bFloorSnappingEnabled ? SpringForce : FMath::Min(0.f, SpringForce);

		if (bFloatingEnabled)
		{
			OwnerPrimitiveCompo->AddForce(TraceDirection * SpringForce * OwnerPrimitiveCompo->GetMass());
		}

		if (OtherComponent != nullptr)
		{
			GroundVelocity = OtherComponent->GetComponentVelocity();

			if (OtherComponent->IsSimulatingPhysics())
			{
				OtherComponent->AddForceAtLocation(TraceDirection * -SpringForce * OtherComponent->GetMass(),
				                                   OutHit.Location);
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
	const AController* Controller = PawnOwner->GetController();
	if (Controller && Controller->IsLocalController())
	{
		const FVector ControlDirection = GetPendingInputVector().GetClampedToMaxSize(1.f);
		
		if(!FMath::IsNearlyZero(ControlDirection.SizeSquared()))
		{
			PawnOrientation = ControlDirection.ToOrientationQuat();
		}
		const FVector OwnerVelocity = OwnerPrimitiveCompo->GetComponentVelocity();
		const float VelocityDot = ControlDirection.GetSafeNormal() | OwnerVelocity;
		
		const float FinalAcceleration = (MaxAccelForce * MaxAccelerationForceFactorFromDot.GetRichCurveConst()->Eval(VelocityDot));
		
		const FVector GoalVelocity = ControlDirection * MaxSpeed;

		FVector NeededAcceleration = ((GoalVelocity + (GroundVelocity * ForceScale)) - (OwnerVelocity * ForceScale)) / DeltaTime;
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
		OnStoppedJumping();
		bCheckForApex = false;
		OnApexReached.Broadcast();
	}
	
	if (bRequestApexCheck)
	{
		bCheckForApex = true;
		bRequestApexCheck = false;
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
	GravityScale = InGravity / GetGravityZ();
	OnGravityChanged.Broadcast(GravityScale * GetGravityZ());
}



