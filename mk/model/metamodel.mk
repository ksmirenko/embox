# Generated by Xpand using M2Make template.

# Meta model for 'EModel' package.

ifndef __model_metamodel_mk
__model_metamodel_mk := $(lastword $(MAKEFILE_LIST))

# Create meta objects.
EModel := \
	$(call createMetaModel,EModel)

EModel_EObject := \
	$(call createMetaClass,$(EModel),EModel_EObject)
EModel_EObject_eMetaClass := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eMetaClass)
EModel_EObject_eResource := \
	$(call createMetaAttribute,$(EModel_EObject),EModel_EObject_eResource)
EModel_EObject_eContainer := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eContainer)
EModel_EObject_eRootContainer := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eRootContainer)
EModel_EObject_eContents := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eContents)
EModel_EObject_eAllContents := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eAllContents)
EModel_EObject_eLinks := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eLinks)
EModel_EObject_eResolvedLinks := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eResolvedLinks)
EModel_EObject_eInverseResolvedLinks := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eInverseResolvedLinks)
EModel_EObject_eUnresolvedLinks := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eUnresolvedLinks)
EModel_EObject_eRefs := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eRefs)
EModel_EObject_eInverseRefs := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eInverseRefs)
EModel_EObject_eLinkedRefs := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eLinkedRefs)
EModel_EObject_eInverseLinkedRefs := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eInverseLinkedRefs)
EModel_EObject_eImmediateRefs := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eImmediateRefs)
EModel_EObject_eInverseImmediateRefs := \
	$(call createMetaReference,$(EModel_EObject),EModel_EObject_eInverseImmediateRefs)

EModel_ELink := \
	$(call createMetaClass,$(EModel),EModel_ELink)
EModel_ELink_eMetaReference := \
	$(call createMetaReference,$(EModel_ELink),EModel_ELink_eMetaReference)
EModel_ELink_eSource := \
	$(call createMetaReference,$(EModel_ELink),EModel_ELink_eSource)
EModel_ELink_eDestination := \
	$(call createMetaReference,$(EModel_ELink),EModel_ELink_eDestination)

EModel_EMetaType := \
	$(call createMetaClass,$(EModel),EModel_EMetaType)
EModel_EMetaType_instanceClass := \
	$(call createMetaAttribute,$(EModel_EMetaType),EModel_EMetaType_instanceClass)
EModel_EMetaType_eMetaModel := \
	$(call createMetaReference,$(EModel_EMetaType),EModel_EMetaType_eMetaModel)

EModel_EMetaClass := \
	$(call createMetaClass,$(EModel),EModel_EMetaClass)
EModel_EMetaClass_isAbstract := \
	$(call createMetaAttribute,$(EModel_EMetaClass),EModel_EMetaClass_isAbstract)
EModel_EMetaClass_isInterface := \
	$(call createMetaAttribute,$(EModel_EMetaClass),EModel_EMetaClass_isInterface)
EModel_EMetaClass_eSuperTypes := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eSuperTypes)
EModel_EMetaClass_eAllSuperTypes := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eAllSuperTypes)
EModel_EMetaClass_eFeatures := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eFeatures)
EModel_EMetaClass_eAllFeatures := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eAllFeatures)
EModel_EMetaClass_eAttributes := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eAttributes)
EModel_EMetaClass_eAllAttributes := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eAllAttributes)
EModel_EMetaClass_eReferences := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eReferences)
EModel_EMetaClass_eAllReferences := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eAllReferences)
EModel_EMetaClass_eAllContainments := \
	$(call createMetaReference,$(EModel_EMetaClass),EModel_EMetaClass_eAllContainments)

EModel_EMetaPrimitive := \
	$(call createMetaClass,$(EModel),EModel_EMetaPrimitive)

EModel_EMetaFeature := \
	$(call createMetaClass,$(EModel),EModel_EMetaFeature)
EModel_EMetaFeature_isChangeable := \
	$(call createMetaAttribute,$(EModel_EMetaFeature),EModel_EMetaFeature_isChangeable)
EModel_EMetaFeature_isDerived := \
	$(call createMetaAttribute,$(EModel_EMetaFeature),EModel_EMetaFeature_isDerived)
EModel_EMetaFeature_instanceProperty := \
	$(call createMetaAttribute,$(EModel_EMetaFeature),EModel_EMetaFeature_instanceProperty)
EModel_EMetaFeature_eContainingClass := \
	$(call createMetaReference,$(EModel_EMetaFeature),EModel_EMetaFeature_eContainingClass)

EModel_EMetaReference := \
	$(call createMetaClass,$(EModel),EModel_EMetaReference)
EModel_EMetaReference_isContainment := \
	$(call createMetaAttribute,$(EModel_EMetaReference),EModel_EMetaReference_isContainment)
EModel_EMetaReference_isContainer := \
	$(call createMetaAttribute,$(EModel_EMetaReference),EModel_EMetaReference_isContainer)
EModel_EMetaReference_eOpposite := \
	$(call createMetaReference,$(EModel_EMetaReference),EModel_EMetaReference_eOpposite)
EModel_EMetaReference_eReferenceType := \
	$(call createMetaReference,$(EModel_EMetaReference),EModel_EMetaReference_eReferenceType)

EModel_EMetaAttribute := \
	$(call createMetaClass,$(EModel),EModel_EMetaAttribute)
EModel_EMetaAttribute_eAttributeType := \
	$(call createMetaReference,$(EModel_EMetaAttribute),EModel_EMetaAttribute_eAttributeType)

EModel_EMetaModel := \
	$(call createMetaClass,$(EModel),EModel_EMetaModel)
EModel_EMetaModel_eTypes := \
	$(call createMetaReference,$(EModel_EMetaModel),EModel_EMetaModel_eTypes)

EModel_ENamed := \
	$(call createMetaClass,$(EModel),EModel_ENamed)
EModel_ENamed_name := \
	$(call createMetaAttribute,$(EModel_ENamed),EModel_ENamed_name)
EModel_ENamed_qualifiedName := \
	$(call createMetaAttribute,$(EModel_ENamed),EModel_ENamed_qualifiedName)

EModel_ETyped := \
	$(call createMetaClass,$(EModel),EModel_ETyped)
EModel_ETyped_isMany := \
	$(call createMetaAttribute,$(EModel_ETyped),EModel_ETyped_isMany)
EModel_ETyped_eType := \
	$(call createMetaReference,$(EModel_ETyped),EModel_ETyped_eType)

# Initializes the objects and relations between them.
define __eModel_init
	$(call initMetaModel,$(EModel),EModel,)

	$(call initMetaClass,$(EModel_EObject),EObject,,)
	$(call initMetaReference,$(EModel_EObject_eMetaClass),eMetaClass,$(EModel_EMetaClass),,derived)
	$(call initMetaAttribute,$(EModel_EObject_eResource),eResource,derived)
	$(call initMetaReference,$(EModel_EObject_eContainer),eContainer,$(EModel_EObject),$(EModel_EObject_eContents),derived)
	$(call initMetaReference,$(EModel_EObject_eRootContainer),eRootContainer,$(EModel_EObject),,derived)
	$(call initMetaReference,$(EModel_EObject_eContents),eContents,$(EModel_EObject),$(EModel_EObject_eContainer),derived many)
	$(call initMetaReference,$(EModel_EObject_eAllContents),eAllContents,$(EModel_EObject),,derived many)
	$(call initMetaReference,$(EModel_EObject_eLinks),eLinks,$(EModel_ELink),$(EModel_ELink_eSource),derived many containment)
	$(call initMetaReference,$(EModel_EObject_eResolvedLinks),eResolvedLinks,$(EModel_ELink),,derived many)
	$(call initMetaReference,$(EModel_EObject_eInverseResolvedLinks),eInverseResolvedLinks,$(EModel_ELink),$(EModel_ELink_eDestination),derived many)
	$(call initMetaReference,$(EModel_EObject_eUnresolvedLinks),eUnresolvedLinks,$(EModel_ELink),,derived many)
	$(call initMetaReference,$(EModel_EObject_eRefs),eRefs,$(EModel_EObject),$(EModel_EObject_eInverseRefs),derived many)
	$(call initMetaReference,$(EModel_EObject_eInverseRefs),eInverseRefs,$(EModel_EObject),$(EModel_EObject_eRefs),derived many)
	$(call initMetaReference,$(EModel_EObject_eLinkedRefs),eLinkedRefs,$(EModel_EObject),$(EModel_EObject_eInverseLinkedRefs),derived many)
	$(call initMetaReference,$(EModel_EObject_eInverseLinkedRefs),eInverseLinkedRefs,$(EModel_EObject),$(EModel_EObject_eLinkedRefs),derived many)
	$(call initMetaReference,$(EModel_EObject_eImmediateRefs),eImmediateRefs,$(EModel_EObject),$(EModel_EObject_eInverseImmediateRefs),derived many)
	$(call initMetaReference,$(EModel_EObject_eInverseImmediateRefs),eInverseImmediateRefs,$(EModel_EObject),$(EModel_EObject_eImmediateRefs),derived many)

	$(call initMetaClass,$(EModel_ELink),ELink,$(EModel_ENamed),)
	$(call initMetaReference,$(EModel_ELink_eMetaReference),eMetaReference,$(EModel_EMetaReference),,changeable)
	$(call initMetaReference,$(EModel_ELink_eSource),eSource,$(EModel_EObject),$(EModel_EObject_eLinks),derived container)
	$(call initMetaReference,$(EModel_ELink_eDestination),eDestination,$(EModel_EObject),$(EModel_EObject_eInverseResolvedLinks),derived)

	$(call initMetaClass,$(EModel_EMetaType),EMetaType,$(EModel_ENamed),abstract)
	$(call initMetaAttribute,$(EModel_EMetaType_instanceClass),instanceClass,changeable)
	$(call initMetaReference,$(EModel_EMetaType_eMetaModel),eMetaModel,$(EModel_EMetaModel),$(EModel_EMetaModel_eTypes),container)

	$(call initMetaClass,$(EModel_EMetaClass),EMetaClass,$(EModel_EMetaType),)
	$(call initMetaAttribute,$(EModel_EMetaClass_isAbstract),abstract,changeable)
	$(call initMetaAttribute,$(EModel_EMetaClass_isInterface),interface,changeable)
	$(call initMetaReference,$(EModel_EMetaClass_eSuperTypes),eSuperTypes,$(EModel_EMetaClass),,changeable many)
	$(call initMetaReference,$(EModel_EMetaClass_eAllSuperTypes),eAllSuperTypes,$(EModel_EMetaClass),,derived many)
	$(call initMetaReference,$(EModel_EMetaClass_eFeatures),eFeatures,$(EModel_EMetaFeature),$(EModel_EMetaFeature_eContainingClass),changeable many containment)
	$(call initMetaReference,$(EModel_EMetaClass_eAllFeatures),eAllFeatures,$(EModel_EMetaFeature),,derived many)
	$(call initMetaReference,$(EModel_EMetaClass_eAttributes),eAttributes,$(EModel_EMetaAttribute),,derived many)
	$(call initMetaReference,$(EModel_EMetaClass_eAllAttributes),eAllAttributes,$(EModel_EMetaAttribute),,derived many)
	$(call initMetaReference,$(EModel_EMetaClass_eReferences),eReferences,$(EModel_EMetaReference),,derived many)
	$(call initMetaReference,$(EModel_EMetaClass_eAllReferences),eAllReferences,$(EModel_EMetaReference),,derived many)
	$(call initMetaReference,$(EModel_EMetaClass_eAllContainments),eAllContainments,$(EModel_EMetaReference),,derived many)

	$(call initMetaClass,$(EModel_EMetaPrimitive),EMetaPrimitive,$(EModel_EMetaType),)

	$(call initMetaClass,$(EModel_EMetaFeature),EMetaFeature,$(EModel_ETyped),abstract)
	$(call initMetaAttribute,$(EModel_EMetaFeature_isChangeable),changeable,changeable)
	$(call initMetaAttribute,$(EModel_EMetaFeature_isDerived),derived,changeable)
	$(call initMetaAttribute,$(EModel_EMetaFeature_instanceProperty),instanceProperty,changeable)
	$(call initMetaReference,$(EModel_EMetaFeature_eContainingClass),eContainingClass,$(EModel_EMetaClass),$(EModel_EMetaClass_eFeatures),container)

	$(call initMetaClass,$(EModel_EMetaReference),EMetaReference,$(EModel_EMetaFeature),)
	$(call initMetaAttribute,$(EModel_EMetaReference_isContainment),containment,changeable)
	$(call initMetaAttribute,$(EModel_EMetaReference_isContainer),container,derived)
	$(call initMetaReference,$(EModel_EMetaReference_eOpposite),eOpposite,$(EModel_EMetaReference),,changeable)
	$(call initMetaReference,$(EModel_EMetaReference_eReferenceType),eReferenceType,$(EModel_EMetaClass),,derived)

	$(call initMetaClass,$(EModel_EMetaAttribute),EMetaAttribute,$(EModel_EMetaFeature),)
	$(call initMetaReference,$(EModel_EMetaAttribute_eAttributeType),eAttributeType,$(EModel_EMetaPrimitive),,derived)

	$(call initMetaClass,$(EModel_EMetaModel),EMetaModel,$(EModel_ENamed),)
	$(call initMetaReference,$(EModel_EMetaModel_eTypes),eTypes,$(EModel_EMetaType),$(EModel_EMetaType_eMetaModel),changeable many containment)

	$(call initMetaClass,$(EModel_ENamed),ENamed,,abstract)
	$(call initMetaAttribute,$(EModel_ENamed_name),name,changeable)
	$(call initMetaAttribute,$(EModel_ENamed_qualifiedName),qualifiedName,derived)

	$(call initMetaClass,$(EModel_ETyped),ETyped,$(EModel_ENamed),abstract)
	$(call initMetaAttribute,$(EModel_ETyped_isMany),many,changeable)
	$(call initMetaReference,$(EModel_ETyped_eType),eType,$(EModel_EMetaType),,changeable)

endef # __eModel_init

# Binds objects to instance classes and features to properties.
define __eModel_bind
	$(call bindMetaClass,$(EModel_EObject),EObject)
	$(call bindMetaFeature,$(EModel_EObject_eMetaClass),eMetaClass)
	$(call bindMetaFeature,$(EModel_EObject_eResource),eResource)
	$(call bindMetaFeature,$(EModel_EObject_eContainer),eContainer)
	$(call bindMetaFeature,$(EModel_EObject_eRootContainer),eRootContainer)
	$(call bindMetaFeature,$(EModel_EObject_eContents),eContents)
	$(call bindMetaFeature,$(EModel_EObject_eAllContents),eAllContents)
	$(call bindMetaFeature,$(EModel_EObject_eLinks),eLinks)
	$(call bindMetaFeature,$(EModel_EObject_eResolvedLinks),eResolvedLinks)
	$(call bindMetaFeature,$(EModel_EObject_eInverseResolvedLinks),eInverseResolvedLinks)
	$(call bindMetaFeature,$(EModel_EObject_eUnresolvedLinks),eUnresolvedLinks)
	$(call bindMetaFeature,$(EModel_EObject_eRefs),eRefs)
	$(call bindMetaFeature,$(EModel_EObject_eInverseRefs),eInverseRefs)
	$(call bindMetaFeature,$(EModel_EObject_eLinkedRefs),eLinkedRefs)
	$(call bindMetaFeature,$(EModel_EObject_eInverseLinkedRefs),eInverseLinkedRefs)
	$(call bindMetaFeature,$(EModel_EObject_eImmediateRefs),eImmediateRefs)
	$(call bindMetaFeature,$(EModel_EObject_eInverseImmediateRefs),eInverseImmediateRefs)

	$(call bindMetaClass,$(EModel_ELink),ELink)
	$(call bindMetaFeature,$(EModel_ELink_eMetaReference),eMetaReference)
	$(call bindMetaFeature,$(EModel_ELink_eSource),eSource)
	$(call bindMetaFeature,$(EModel_ELink_eDestination),eDestination)

	$(call bindMetaClass,$(EModel_EMetaType),EMetaType)
	$(call bindMetaFeature,$(EModel_EMetaType_instanceClass),instanceClass)
	$(call bindMetaFeature,$(EModel_EMetaType_eMetaModel),eMetaModel)

	$(call bindMetaClass,$(EModel_EMetaClass),EMetaClass)
	$(call bindMetaFeature,$(EModel_EMetaClass_isAbstract),isAbstract)
	$(call bindMetaFeature,$(EModel_EMetaClass_isInterface),isInterface)
	$(call bindMetaFeature,$(EModel_EMetaClass_eSuperTypes),eSuperTypes)
	$(call bindMetaFeature,$(EModel_EMetaClass_eAllSuperTypes),eAllSuperTypes)
	$(call bindMetaFeature,$(EModel_EMetaClass_eFeatures),eFeatures)
	$(call bindMetaFeature,$(EModel_EMetaClass_eAllFeatures),eAllFeatures)
	$(call bindMetaFeature,$(EModel_EMetaClass_eAttributes),eAttributes)
	$(call bindMetaFeature,$(EModel_EMetaClass_eAllAttributes),eAllAttributes)
	$(call bindMetaFeature,$(EModel_EMetaClass_eReferences),eReferences)
	$(call bindMetaFeature,$(EModel_EMetaClass_eAllReferences),eAllReferences)
	$(call bindMetaFeature,$(EModel_EMetaClass_eAllContainments),eAllContainments)

	$(call bindMetaClass,$(EModel_EMetaPrimitive),EMetaPrimitive)

	$(call bindMetaClass,$(EModel_EMetaFeature),EMetaFeature)
	$(call bindMetaFeature,$(EModel_EMetaFeature_isChangeable),isChangeable)
	$(call bindMetaFeature,$(EModel_EMetaFeature_isDerived),isDerived)
	$(call bindMetaFeature,$(EModel_EMetaFeature_instanceProperty),instanceProperty)
	$(call bindMetaFeature,$(EModel_EMetaFeature_eContainingClass),eContainingClass)

	$(call bindMetaClass,$(EModel_EMetaReference),EMetaReference)
	$(call bindMetaFeature,$(EModel_EMetaReference_isContainment),isContainment)
	$(call bindMetaFeature,$(EModel_EMetaReference_isContainer),isContainer)
	$(call bindMetaFeature,$(EModel_EMetaReference_eOpposite),eOpposite)
	$(call bindMetaFeature,$(EModel_EMetaReference_eReferenceType),eReferenceType)

	$(call bindMetaClass,$(EModel_EMetaAttribute),EMetaAttribute)
	$(call bindMetaFeature,$(EModel_EMetaAttribute_eAttributeType),eAttributeType)

	$(call bindMetaClass,$(EModel_EMetaModel),EMetaModel)
	$(call bindMetaFeature,$(EModel_EMetaModel_eTypes),eTypes)

	$(call bindMetaClass,$(EModel_ENamed),ENamed)
	$(call bindMetaFeature,$(EModel_ENamed_name),name)
	$(call bindMetaFeature,$(EModel_ENamed_qualifiedName),qualifiedName)

	$(call bindMetaClass,$(EModel_ETyped),ETyped)
	$(call bindMetaFeature,$(EModel_ETyped_isMany),isMany)
	$(call bindMetaFeature,$(EModel_ETyped_eType),eType)

endef # __eModel_bind

$(def_all)

$(call __eModel_init)
$(call __eModel_bind)

endif # __model_metamodel_mk

