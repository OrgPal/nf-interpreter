//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include "stdafx.h"

#include "Core.h"
#include "corhdr_private.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

#define ITERATE_THROUGH_RECORDS(assm, i, tblName, tblNameUC)                                                           \
    const CLR_RECORD_##tblNameUC *src = (const CLR_RECORD_##tblNameUC *)assm->GetTable(TBL_##tblName);                 \
    CLR_RT_##tblName##_CrossReference *dst = assm->m_pCrossReference_##tblName;                                        \
    for (i = 0; i < assm->m_pTablesSize[TBL_##tblName]; i++, src++, dst++)

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
#define NANOCLR_TRACE_DEFAULT(win, arm) (win)
#else
#define NANOCLR_TRACE_DEFAULT(win, arm) (arm)
#endif

#if defined(NANOCLR_TRACE_ERRORS)
int s_CLR_RT_fTrace_Errors = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_Info, c_CLR_RT_Trace_Info);
#endif

#if defined(NANOCLR_TRACE_EXCEPTIONS)
int s_CLR_RT_fTrace_Exceptions = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_Info, c_CLR_RT_Trace_Info);
#endif

#if defined(NANOCLR_TRACE_INSTRUCTIONS)
int s_CLR_RT_fTrace_Instructions = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_None, c_CLR_RT_Trace_None);
#endif

#if defined(NANOCLR_GC_VERBOSE)
int s_CLR_RT_fTrace_Memory = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_None, c_CLR_RT_Trace_None);
#endif

#if defined(NANOCLR_TRACE_MEMORY_STATS)
int s_CLR_RT_fTrace_MemoryStats = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_Info, c_CLR_RT_Trace_Info);
#endif

#if defined(NANOCLR_GC_VERBOSE)
int s_CLR_RT_fTrace_GC = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_None, c_CLR_RT_Trace_None);
#endif

#if defined(WIN32)
int s_CLR_RT_fTrace_SimulateSpeed = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_Info, c_CLR_RT_Trace_None);
#endif

#if !defined(BUILD_RTM)
int s_CLR_RT_fTrace_AssemblyOverhead = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_Info, c_CLR_RT_Trace_Info);
#endif

#if defined(WIN32)
int s_CLR_RT_fTrace_StopOnFAILED = NANOCLR_TRACE_DEFAULT(c_CLR_RT_Trace_None, c_CLR_RT_Trace_None);
#endif

#if defined(WIN32)
int s_CLR_RT_fTrace_ARM_Execution = 0;

int s_CLR_RT_fTrace_RedirectLinesPerFile;
std::wstring s_CLR_RT_fTrace_RedirectOutput;
std::wstring s_CLR_RT_fTrace_RedirectCallChain;

std::wstring s_CLR_RT_fTrace_HeapDump_FilePrefix;
bool s_CLR_RT_fTrace_HeapDump_IncludeCreators = false;

bool s_CLR_RT_fTimeWarp = false;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

void CLR_RT_ReflectionDef_Index::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    m_kind = REFLECTION_INVALID;
    m_levels = 0;
    m_data.m_raw = 0;
}

CLR_UINT32 CLR_RT_ReflectionDef_Index::GetTypeHash() const
{
    NATIVE_PROFILE_CLR_CORE();
    switch (m_kind)
    {
        case REFLECTION_TYPE:
        {
            CLR_RT_TypeDef_Instance inst;

            if (m_levels != 0)
                return 0;

            if (!inst.InitializeFromIndex(m_data.m_type))
                return 0;

            return inst.CrossReference().m_hash;
        }

        case REFLECTION_TYPE_DELAYED:
            return m_data.m_raw;
    }

    return 0;
}

void CLR_RT_ReflectionDef_Index::InitializeFromHash(CLR_UINT32 hash)
{
    NATIVE_PROFILE_CLR_CORE();
    m_kind = REFLECTION_TYPE_DELAYED;
    m_levels = 0;
    m_data.m_raw = hash;
}

CLR_UINT64 CLR_RT_ReflectionDef_Index::GetRawData() const
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_UINT64 data;
    _ASSERTE(sizeof(data) == sizeof(*this));

    memcpy(&data, this, sizeof(data));

    return data;
}

void CLR_RT_ReflectionDef_Index::SetRawData(CLR_UINT64 data)
{
    NATIVE_PROFILE_CLR_CORE();
    _ASSERTE(sizeof(data) == sizeof(*this));

    memcpy(this, &data, sizeof(data));
}

bool CLR_RT_ReflectionDef_Index::Convert(CLR_RT_HeapBlock &ref, CLR_RT_Assembly_Instance &inst)
{
    NATIVE_PROFILE_CLR_CORE();
    if (ref.DataType() == DATATYPE_REFLECTION)
    {
        return inst.InitializeFromIndex(ref.ReflectionDataConst().m_data.m_assm);
    }

    return false;
}

bool CLR_RT_ReflectionDef_Index::Convert(CLR_RT_HeapBlock &ref, CLR_RT_TypeDef_Instance &inst, CLR_UINT32 *levels)
{
    NATIVE_PROFILE_CLR_CORE();
    if (ref.DataType() == DATATYPE_REFLECTION)
    {
        return inst.InitializeFromReflection(ref.ReflectionDataConst(), levels);
    }

    return false;
}

bool CLR_RT_ReflectionDef_Index::Convert(CLR_RT_HeapBlock &ref, CLR_RT_MethodDef_Instance &inst)
{
    NATIVE_PROFILE_CLR_CORE();
    if (ref.DataType() == DATATYPE_REFLECTION)
    {
        switch (ref.ReflectionData().m_kind)
        {
            case REFLECTION_CONSTRUCTOR:
            case REFLECTION_METHOD:
                return inst.InitializeFromIndex(ref.ReflectionDataConst().m_data.m_method);
        }
    }

    return false;
}

bool CLR_RT_ReflectionDef_Index::Convert(CLR_RT_HeapBlock &ref, CLR_RT_FieldDef_Instance &inst)
{
    NATIVE_PROFILE_CLR_CORE();
    if (ref.DataType() == DATATYPE_REFLECTION && ref.ReflectionData().m_kind == REFLECTION_FIELD)
    {
        return inst.InitializeFromIndex(ref.ReflectionDataConst().m_data.m_field);
    }

    return false;
}

bool CLR_RT_ReflectionDef_Index::Convert(CLR_RT_HeapBlock &ref, CLR_UINT32 &hash)
{
    NATIVE_PROFILE_CLR_CORE();
    if (ref.DataType() != DATATYPE_REFLECTION)
        return false;

    hash = ref.ReflectionData().GetTypeHash();

    return hash != 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CLR_RT_SignatureParser::Initialize_TypeSpec(CLR_RT_Assembly *assm, const CLR_RECORD_TYPESPEC *ts)
{
    NATIVE_PROFILE_CLR_CORE();
    Initialize_TypeSpec(assm, assm->GetSignature(ts->Sig));
}

void CLR_RT_SignatureParser::Initialize_TypeSpec(CLR_RT_Assembly *assm, CLR_PMETADATA ts)
{
    NATIVE_PROFILE_CLR_CORE();
    Assembly = assm;
    Signature = ts;

    Type = CLR_RT_SignatureParser::c_TypeSpec;
    Flags = 0;
    ParamCount = 1;
    GenParamCount = 0;
    IsGenericInst = false;
}

//--//

void CLR_RT_SignatureParser::Initialize_Interfaces(CLR_RT_Assembly *assm, const CLR_RECORD_TYPEDEF *td)
{
    NATIVE_PROFILE_CLR_CORE();
    if (td->Interfaces != CLR_EmptyIndex)
    {
        CLR_PMETADATA sig = assm->GetSignature(td->Interfaces);

        ParamCount = (*sig++);
        Signature = sig;
    }
    else
    {
        ParamCount = 0;
        Signature = NULL;
    }

    Type = CLR_RT_SignatureParser::c_Interfaces;
    Flags = 0;

    Assembly = assm;

    GenParamCount = 0;
    IsGenericInst = false;
}

//--//

void CLR_RT_SignatureParser::Initialize_FieldSignature(CLR_RT_Assembly* assm, const CLR_RECORD_FIELDDEF* fd)
{
    NATIVE_PROFILE_CLR_CORE();

    Method = 0xFFFF;

    Initialize_FieldSignature(assm, assm->GetSignature(fd->Sig));
}

void CLR_RT_SignatureParser::Initialize_FieldSignature(CLR_RT_Assembly* assm, CLR_PMETADATA fd)
{
    NATIVE_PROFILE_CLR_CORE();

    Type = CLR_RT_SignatureParser::c_Field;

    Flags = (*fd++);

    ParamCount = 1;

    Assembly = assm;
    Signature = fd;

    GenParamCount = 0;
    IsGenericInst = false;
}

void CLR_RT_SignatureParser::Initialize_FieldDef(CLR_RT_Assembly *assm, const CLR_RECORD_FIELDDEF *fd)
{
    NATIVE_PROFILE_CLR_CORE();
    Initialize_FieldDef(assm, assm->GetSignature(fd->Sig));
}

void CLR_RT_SignatureParser::Initialize_FieldDef(CLR_RT_Assembly *assm, CLR_PMETADATA fd)
{
    NATIVE_PROFILE_CLR_CORE();
    Type = CLR_RT_SignatureParser::c_Field;
    Flags = (*fd++);
    ParamCount = 1;

    Assembly = assm;
    Signature = fd;

    GenParamCount = 0;
    IsGenericInst = false;
}

//--//
void CLR_RT_SignatureParser::Initialize_MethodSignature(CLR_RT_MethodDef_Instance* md)
{
    NATIVE_PROFILE_CLR_CORE();

    Method = md->Method();

    Initialize_MethodSignature(md->m_assm, md->m_assm->GetSignature(md->m_target->Sig));
}

void CLR_RT_SignatureParser::Initialize_MethodSignature(CLR_RT_Assembly* assm, const CLR_RECORD_METHODDEF* md)
{
    NATIVE_PROFILE_CLR_CORE();

    Method = 0xFFFF;

    Initialize_MethodSignature(assm, assm->GetSignature(md->Sig));
}

void CLR_RT_SignatureParser::Initialize_MethodSignature(CLR_RT_Assembly *assm, CLR_PMETADATA md)
{
    NATIVE_PROFILE_CLR_CORE();

    Type = CLR_RT_SignatureParser::c_Method;

    Flags = (*md++);

    if ((Flags & PIMAGE_CEE_CS_CALLCONV_GENERIC) == PIMAGE_CEE_CS_CALLCONV_GENERIC)
    {
        // is generic instance, has generic parameters count
        GenParamCount = (*md++);
    }
    else
    {
        GenParamCount = 0;
    }

    ParamCount = (*md++) + 1;

    Assembly = assm;
    Signature = md;

    GenParamCount = 0;
    IsGenericInst = false;
}

void CLR_RT_SignatureParser::Initialize_MethodSignature(CLR_RT_MethodSpec_Instance* ms)
{
    NATIVE_PROFILE_CLR_CORE();

    Method = ms->Method();

    Signature = ms->m_assm->GetSignature(ms->m_target->Instantiation);

    Type = CLR_RT_SignatureParser::c_MethodSpec;

    Flags = (*Signature++);

    if (Flags != PIMAGE_CEE_CS_CALLCONV_GENERICINST)
    {
        // call is wrong
        return;
    }

    ParamCount = (*Signature++);

    Assembly = ms->m_assm;

    GenParamCount = ParamCount;

    IsGenericInst = false;
}

//--//

bool CLR_RT_SignatureParser::Initialize_GenericParamTypeSignature(CLR_RT_Assembly* assm, const CLR_RECORD_GENERICPARAM* gp)
{
    NATIVE_PROFILE_CLR_CORE();

    Type = CLR_RT_SignatureParser::c_GenericParamType;

    Assembly = assm;
    
    // need to check for valid signature
    if (gp->Sig != 0xFFFF)
    {
        Signature = assm->GetSignature(gp->Sig);
        ParamCount = 1;
    }
    else
    {
        // can't process...
        return false;
    }

    Flags = 0;

    GenParamCount = 0;
    IsGenericInst = false;

    // done here
    return true;
}

//--//

void CLR_RT_SignatureParser::Initialize_MethodLocals(CLR_RT_Assembly *assm, const CLR_RECORD_METHODDEF *md)
{
    NATIVE_PROFILE_CLR_CORE();
    //
    // WARNING!!!
    //
    // If you change this method, change "CLR_RT_ExecutionEngine::InitializeLocals" too.
    //

    Assembly = assm;
    Signature = assm->GetSignature(md->Locals);

    Type = CLR_RT_SignatureParser::c_Locals;
    Flags = 0;
    ParamCount = md->LocalsCount;

    GenParamCount = 0;
    IsGenericInst = false;
}

//--//

void CLR_RT_SignatureParser::Initialize_Objects(CLR_RT_HeapBlock *lst, int count, bool fTypes)
{
    NATIVE_PROFILE_CLR_CORE();
    ObjectList = lst;

    Type = CLR_RT_SignatureParser::c_Object;
    Flags = fTypes ? 1 : 0;
    ParamCount = count;

    GenParamCount = 0;
    IsGenericInst = false;
}

//--//

HRESULT CLR_RT_SignatureParser::Advance(Element &res)
{
    NATIVE_PROFILE_CLR_CORE();
    //
    // WARNING!!!
    //
    // If you change this method, change "CLR_RT_ExecutionEngine::InitializeLocals" too.
    //

    NANOCLR_HEADER();

    _ASSERTE(ParamCount > 0);

    ParamCount--;

    res.IsByRef = false;
    res.Levels = 0;
    res.GenericParamPosition = 0xFFFF;

    switch (Type)
    {
        case c_Interfaces:
        {
            CLR_RT_TypeDef_Instance cls;

            res.DataType = DATATYPE_CLASS;

            if (cls.ResolveToken(CLR_TkFromStream(Signature), Assembly) == false)
            {
                NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
            }

            res.Class = cls;
        }
        break;

        case c_Object:
        {
            CLR_RT_TypeDescriptor desc;
            CLR_RT_HeapBlock *ptr = ObjectList++;

            if (Flags)
            {
                // Reflection types are now boxed, so unbox first
                if (ptr->DataType() == DATATYPE_OBJECT)
                {
                    ptr = ptr->Dereference();
                }
                if (ptr->DataType() != DATATYPE_REFLECTION)
                {
                    NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
                }

                NANOCLR_CHECK_HRESULT(desc.InitializeFromReflection(ptr->ReflectionDataConst()));
            }
            else
            {
                switch (ptr->DataType())
                {
                    case DATATYPE_BYREF:
                    case DATATYPE_ARRAY_BYREF:
                        res.IsByRef = true;
                        break;

                    default:
                        // the remaining data types aren't to be handled
                        break;
                }

                NANOCLR_CHECK_HRESULT(desc.InitializeFromObject(*ptr));
            }

            desc.m_handlerCls.InitializeFromIndex(desc.m_reflex.m_data.m_type);

            res.Levels = desc.m_reflex.m_levels;
            res.DataType = (nanoClrDataType)desc.m_handlerCls.m_target->DataType;
            res.Class = desc.m_reflex.m_data.m_type;

            //
            // Special case for Object types.
            //
            if (res.Class.m_data == g_CLR_RT_WellKnownTypes.m_Object.m_data)
            {
                res.DataType = DATATYPE_OBJECT;
            }
        }
        break;

        default:
            while (true)
            {
                res.DataType = CLR_UncompressElementType(Signature);

                switch (res.DataType)
                {
                    case DATATYPE_BYREF:
                        if (res.IsByRef)
                        {
                            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
                        }

                        res.IsByRef = true;
                        break;

                    case DATATYPE_SZARRAY:
                        res.Levels++;
                        break;

                    case DATATYPE_CLASS:
                    case DATATYPE_VALUETYPE:
                    {
parse_type:
                        CLR_UINT32 tk = CLR_TkFromStream(Signature);
                        CLR_UINT32 index = CLR_DataFromTk(tk);

                        switch (CLR_TypeFromTk(tk))
                        {
                            case TBL_TypeSpec:
                            {
                                CLR_RT_SignatureParser sub;
                                sub.Initialize_TypeSpec(Assembly, Assembly->GetTypeSpec(index));
                                CLR_RT_SignatureParser::Element res;
                                int extraLevels = res.Levels;

                                NANOCLR_CHECK_HRESULT(sub.Advance(res));

                                res.Levels += extraLevels;
                            }
                            break;

                            case TBL_TypeRef:
                                res.Class = Assembly->m_pCrossReference_TypeRef[index].m_target;
                                break;

                            case TBL_TypeDef:
                                CLR_RT_TypeDef_Instance cls;

                                if (cls.ResolveToken(tk, Assembly) == false)
                                {
                                    NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
                                }

                                res.Class = cls;
                                break;

                            default:
                                NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
                        }

                        if (IsGenericInst)
                        {
                            // get generic arguments count
                            GenParamCount = (int)*Signature++;

                            // need to update the parser counter too
                            ParamCount = GenParamCount;
                        }

                        NANOCLR_SET_AND_LEAVE(S_OK);
                    }

                    case DATATYPE_OBJECT:
                        res.Class = g_CLR_RT_WellKnownTypes.m_Object;
                        NANOCLR_SET_AND_LEAVE(S_OK);

                    case DATATYPE_VOID:
                        res.Class = g_CLR_RT_WellKnownTypes.m_Void;
                        NANOCLR_SET_AND_LEAVE(S_OK);

                    case DATATYPE_GENERICINST:
                    {
                        // set flag for GENERICINST 
                        IsGenericInst = true;

                        // get data type
                        nanoClrDataType dt = (nanoClrDataType)*Signature++;

                        // sanity check
                        if (dt == DATATYPE_CLASS ||
                            dt == DATATYPE_VALUETYPE)
                        {
                            // parse type
                            goto parse_type;
                        }
                        else
                        {
                            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
                        }

                        break;
                    }

                    case DATATYPE_VAR:
                        res.GenericParamPosition = (int)*Signature++;
                        NANOCLR_SET_AND_LEAVE(S_OK);

                    case DATATYPE_MVAR:
                        res.GenericParamPosition = (int)*Signature++;
                        NANOCLR_SET_AND_LEAVE(S_OK);

                    default:
                    {
                        const CLR_RT_TypeDef_Index* cls = c_CLR_RT_DataTypeLookup[res.DataType].m_cls;

                        if (cls)
                        {
                            res.Class = *cls;
                            NANOCLR_SET_AND_LEAVE(S_OK);
                        }
                        else
                        {
                            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
                        }
                    }
                }
            }
            break;
    }

    NANOCLR_NOCLEANUP();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CLR_RT_Assembly_Instance::InitializeFromIndex(const CLR_RT_Assembly_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];

        return true;
    }

    m_data = 0;
    m_assm = NULL;

    return false;
}

void CLR_RT_Assembly_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_Assembly_Index::Clear();

    m_assm = NULL;
}

//////////////////////////////

bool CLR_RT_TypeSpec_Instance::InitializeFromIndex(const CLR_RT_TypeSpec_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        m_target = m_assm->GetTypeSpec(TypeSpec());

        CLR_RT_SignatureParser parser;
        parser.Initialize_TypeSpec(m_assm, m_assm->GetTypeSpec(index.TypeSpec()));

        CLR_RT_SignatureParser::Element element;

        // get type
        parser.Advance(element);

        TypeDefIndex = element.Class.Type();

        return true;
    }

    m_data = 0;
    m_assm = NULL;
    m_target = NULL;

    return false;
}

void CLR_RT_TypeSpec_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_TypeSpec_Index::Clear();

    m_assm = NULL;
    m_target = NULL;
}

bool CLR_RT_TypeSpec_Instance::ResolveToken(CLR_UINT32 tk, CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    if (assm && CLR_TypeFromTk(tk) == TBL_TypeSpec)
    {
        CLR_UINT32 index = CLR_DataFromTk(tk);

        Set(assm->m_index, index);

        m_assm = assm;
        m_target = assm->GetTypeSpec(index);

        return true;
    }

    Clear();

    return false;
}

//////////////////////////////

bool CLR_RT_TypeDef_Instance::InitializeFromReflection(const CLR_RT_ReflectionDef_Index &reflex, CLR_UINT32 *levels)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_TypeDef_Index cls;
    const CLR_RT_TypeDef_Index *ptr = NULL;

    if (levels)
        *levels = reflex.m_levels;

    switch (reflex.m_kind)
    {
        case REFLECTION_TYPE:
            if (reflex.m_levels > 0 && levels == NULL)
            {
                ptr = &g_CLR_RT_WellKnownTypes.m_Array;
            }
            else
            {
                ptr = &reflex.m_data.m_type;
            }
            break;

        case REFLECTION_TYPE_DELAYED:
            if (g_CLR_RT_TypeSystem.FindTypeDef(reflex.m_data.m_raw, cls))
            {
                ptr = &cls;
            }
            break;
    }

    return ptr ? InitializeFromIndex(*ptr) : false;
}

bool CLR_RT_TypeDef_Instance::InitializeFromIndex(const CLR_RT_TypeDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        m_target = m_assm->GetTypeDef(Type());

        return true;
    }

    m_data = 0;
    m_assm = NULL;
    m_target = NULL;

    return false;
}

bool CLR_RT_TypeDef_Instance::InitializeFromMethod(const CLR_RT_MethodDef_Instance &md)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(md))
    {
        CLR_INDEX indexAssm = md.Assembly();
        CLR_INDEX indexType = md.CrossReference().GetOwner();

        Set(indexAssm, indexType);

        m_assm = g_CLR_RT_TypeSystem.m_assemblies[indexAssm - 1];
        m_target = m_assm->GetTypeDef(indexType);

        return true;
    }

    Clear();

    return false;
}

bool CLR_RT_TypeDef_Instance::InitializeFromMethod(const CLR_RT_MethodSpec_Instance &ms)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(ms))
    {
        CLR_INDEX indexAssm = ms.Assembly();
       // CLR_INDEX indexType = ms.CrossReference().GetOwner();

        //Set(indexAssm, indexType);

        m_assm = g_CLR_RT_TypeSystem.m_assemblies[indexAssm - 1];
        //m_target = m_assm->GetTypeDef(indexType);

        return true;
    }

    Clear();

    return false;
}

bool CLR_RT_TypeDef_Instance::InitializeFromField(const CLR_RT_FieldDef_Instance &fd)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(fd))
    {
        CLR_RT_Assembly *assm = fd.m_assm;
        const CLR_RECORD_TYPEDEF *td = (const CLR_RECORD_TYPEDEF *)assm->GetTable(TBL_TypeDef);
        CLR_INDEX indexField = fd.Field();
        int i = assm->m_pTablesSize[TBL_TypeDef];

        if (fd.m_target->Flags & CLR_RECORD_FIELDDEF::FD_Static)
        {
            for (; i; i--, td++)
            {
                if (td->FirstStaticField <= indexField && indexField < td->FirstStaticField + td->StaticFieldsCount)
                {
                    break;
                }
            }
        }
        else
        {
            for (; i; i--, td++)
            {
                if (td->FirstInstanceField <= indexField &&
                    indexField < td->FirstInstanceField + td->InstanceFieldsCount)
                {
                    break;
                }
            }
        }

        if (i)
        {
            CLR_INDEX indexAssm = fd.Assembly();
            CLR_INDEX indexType = assm->m_pTablesSize[TBL_TypeDef] - i;

            Set(indexAssm, indexType);

            m_assm = g_CLR_RT_TypeSystem.m_assemblies[indexAssm - 1];
            m_target = m_assm->GetTypeDef(indexType);

            return true;
        }
    }

    Clear();

    return false;
}

bool CLR_RT_TypeDef_Instance::IsATypeHandler()
{
    NATIVE_PROFILE_CLR_CORE();
    return (m_data == g_CLR_RT_WellKnownTypes.m_Type.m_data || m_data == g_CLR_RT_WellKnownTypes.m_TypeStatic.m_data);
}

void CLR_RT_TypeDef_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_TypeDef_Index::Clear();

    m_assm = NULL;
    m_target = NULL;
}

// if type token is not generic, we are going to resolve from the assembly else from the heapblock that may contains
// generic parameter
bool CLR_RT_TypeDef_Instance::ResolveToken(CLR_UINT32 tk, CLR_RT_Assembly *assm, const CLR_RT_MethodDef_Instance *caller)
{
    NATIVE_PROFILE_CLR_CORE();
    if (assm)
    {
        CLR_UINT32 index = CLR_DataFromTk(tk);

        switch (CLR_TypeFromTk(tk))
        {
            case TBL_TypeRef:
                m_data = assm->m_pCrossReference_TypeRef[index].m_target.m_data;
                m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
                m_target = assm->GetTypeDef(Type());
                return true;

            case TBL_TypeDef:
                Set(assm->m_index, index);

                m_assm = assm;
                m_target = assm->GetTypeDef(index);
                return true;

            case TBL_GenericParam:
            {
                CLR_RT_GenericParam_CrossReference gp = assm->m_pCrossReference_GenericParam[index];
               
                Set(gp.Class.Assembly(), gp.Class.Type());

                m_assm = g_CLR_RT_TypeSystem.m_assemblies[gp.Class.Assembly() - 1];
                m_target = m_assm->GetTypeDef(gp.Class.Type());

                return true;
            }

            case TBL_TypeSpec:
            {
                const CLR_RECORD_TYPESPEC* ts = assm->GetTypeSpec(index);

                CLR_RT_SignatureParser parser;
                parser.Initialize_TypeSpec(assm, ts);

                CLR_RT_SignatureParser::Element element;
                
                // advance signature: get parameter
                parser.Advance(element);

                // store parameter position
                CLR_INT8 genericParamPosition = (CLR_INT8)element.GenericParamPosition;

                // get the caller generic type
                CLR_RT_TypeSpec_Instance tsInstance;
                tsInstance.InitializeFromIndex(*caller->genericType);

                switch (element.DataType)
                {
                    case DATATYPE_VAR:

                        parser.Initialize_TypeSpec(caller->m_assm, tsInstance.m_target);

                        // advance signature: get type
                        parser.Advance(element);

                        // loop as many times as the parameter position
                        do
                        {
                            parser.Advance(element);

                            genericParamPosition--;
                        } while (genericParamPosition > 0);

                        // build TypeDef instance from element
                        m_data = element.Class.m_data;
                        m_assm = g_CLR_RT_TypeSystem.m_assemblies[element.Class.Assembly() - 1];
                        m_target = m_assm->GetTypeDef(element.Class.Type());

                        break;

                    case DATATYPE_MVAR:
                        break;

                    default:
                        return false;
                }

                return true;
            }

            default:
                //// handle generic type from provided data
                //if (sampleData != NULL)
                //{
                //    CLR_RT_TypeDescriptor::ExtractTypeIndexFromObject(*sampleData, *this);
                //    m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
                //    m_target = m_assm->GetTypeDef(Type());
                //}
                //else
                //{
                //    m_data = g_CLR_RT_WellKnownTypes.m_Object.m_data;
                //    m_assm = g_CLR_RT_TypeSystem.m_assemblies[g_CLR_RT_WellKnownTypes.m_Object.Assembly() - 1];
                //    m_target = m_assm->GetTypeDef(g_CLR_RT_WellKnownTypes.m_Object.Type());
                //}
                return true;
                // the remaining data types aren't to be handled
                break;
        }
    }

    Clear();

    return false;
}

//--//

bool CLR_RT_TypeDef_Instance::SwitchToParent()
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(*this))
    {
        if (m_target->HasValidExtendsType())
        {
            CLR_RT_TypeDef_Index tmp;
            const CLR_RT_TypeDef_Index *cls;

            switch (m_target->Extends())
            {
                case TBL_TypeDef:
                    tmp.Set(Assembly(), m_target->ExtendsIndex());
                    cls = &tmp;
                    break;

                case TBL_TypeRef:
                    cls = &m_assm->m_pCrossReference_TypeRef[m_target->ExtendsIndex()].m_target;
                    break;

                // all others are not supported
                default:
                    return false;
            }

            return InitializeFromIndex(*cls);
        }
    }

    Clear();

    return false;
}

bool CLR_RT_TypeDef_Instance::HasFinalizer() const
{
    NATIVE_PROFILE_CLR_CORE();
    return NANOCLR_INDEX_IS_VALID(*this) &&
           (CrossReference().m_flags & CLR_RT_TypeDef_CrossReference::TD_CR_HasFinalizer);
}

//////////////////////////////

bool CLR_RT_FieldDef_Instance::InitializeFromIndex(const CLR_RT_FieldDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        m_target = m_assm->GetFieldDef(Field());

        return true;
    }

    m_data = 0;
    m_assm = NULL;
    m_target = NULL;

    return false;
}

void CLR_RT_FieldDef_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_FieldDef_Index::Clear();

    m_assm = NULL;
    m_target = NULL;
}

bool CLR_RT_FieldDef_Instance::ResolveToken(CLR_UINT32 tk, CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    if (assm)
    {
        CLR_UINT32 index = CLR_DataFromTk(tk);

        switch (CLR_TypeFromTk(tk))
        {
            case TBL_FieldRef:
                m_data = assm->m_pCrossReference_FieldRef[index].m_target.m_data;
                m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
                m_target = m_assm->GetFieldDef(Field());
                return true;

            case TBL_FieldDef:
                Set(assm->m_index, index);

                m_assm = assm;
                m_target = m_assm->GetFieldDef(index);
                return true;

            default:
                // the remaining data types aren't to be handled
                break;
        }
    }

    Clear();

    return false;
}

//////////////////////////////

bool CLR_RT_MethodDef_Instance::InitializeFromIndex(const CLR_RT_MethodDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        m_target = m_assm->GetMethodDef(Method());
        genericType = NULL;

        return true;
    }

    m_data = 0;
    m_assm = NULL;
    m_target = NULL;
    genericType = NULL;

    return false;
}

void CLR_RT_MethodDef_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_MethodDef_Index::Clear();

    m_assm = NULL;
    m_target = NULL;
}

bool CLR_RT_MethodDef_Instance::ResolveToken(CLR_UINT32 tk, CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    if (assm)
    {
        CLR_UINT32 index = CLR_DataFromTk(tk);

        switch (CLR_TypeFromTk(tk))
        {
            case TBL_MethodRef:
            {
                const CLR_RECORD_METHODREF* mr = assm->GetMethodRef(index);

                if (mr->Owner() == TBL_TypeSpec)
                {
                    genericType = &assm->m_pCrossReference_MethodRef[index].GenericType;
                    
                    const CLR_RECORD_TYPESPEC *ts = assm->GetTypeSpec(genericType->TypeSpec());

                    CLR_RT_MethodDef_Index method;

                    if (!assm->FindMethodDef(
                        ts, 
                        assm->GetString(mr->Name), 
                        assm, 
                        mr->Sig, method))
                    {
                        return false;
                    }

                    Set(assm->m_index, method.Method());
                    
                    m_assm = assm;
                    m_target = m_assm->GetMethodDef(method.Method());
                }
                else
                {
                    m_data = assm->m_pCrossReference_MethodRef[index].Target.m_data;
                    m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
                    m_target = m_assm->GetMethodDef(Method());

                    // invalidate GenericType
                    genericType = NULL;
                }

                return true;
            }

            case TBL_MethodDef:
                Set(assm->m_index, index);

                m_assm = assm;
                m_target = m_assm->GetMethodDef(index);

                // invalidate generic type
                genericType = NULL;

                return true;

            case TBL_MethodSpec:
            {
                m_assm = assm;

                const CLR_RECORD_METHODSPEC* ms = m_assm->GetMethodSpec(index);

                CLR_RT_MethodSpec_Index msIndex;
                msIndex.Set(m_assm->m_index, index);

                switch (msIndex.Type())
                {
                    case TBL_MethodDef:
                        Set(m_assm->m_index, msIndex.Method());
                        m_assm = assm;
                        m_target = m_assm->GetMethodDef(index);
                        break;

                    case TBL_MethodRef:
                        m_data = assm->m_pCrossReference_MethodRef[msIndex.Method()].Target.m_data;
                        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
                        m_target = m_assm->GetMethodDef(Method());

                        break;

                    default:
                        break;
                }

                // get generic type
                CLR_RT_TypeSpec_Index ts;
                ts.Set(m_assm->m_index, ms->Container);

                //genericType = &m_assm->m_pCrossReference_TypeSpec[ms->Container];

                return true;
            }
            case TBL_TypeSpec:
            {
                //CLR_RT_TypeSpec_Index typeSpec;
                //typeSpec.Set(assm->m_index, index);

                ////CLR_RT_TypeSpec_Instance typeSpecInstance;
                ////typeSpecInstance.InitializeFromIndex(typeSpec);

                CLR_RT_MethodSpec_Index methodSpec;
                assm->FindMethodSpecFromTypeSpec(index, methodSpec);

                switch (methodSpec.Type())
                {
                    case TBL_MethodDef:
                        Set(assm->m_index, methodSpec.Method());

                        m_assm = assm;
                        m_target = m_assm->GetMethodDef(methodSpec.Method());

                        return true;

                    case TBL_MethodRef:
                        m_data = assm->m_pCrossReference_MethodRef[methodSpec.Method()].Target.m_data;
                        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
                        m_target = m_assm->GetMethodDef(Method());

                        return true;

                    default:
                        // shouldn't be here
                        break;
                }
                break;

                return true;
            }
            default:
                // the remaining data types aren't to be handled
                break;
        }
    }

    Clear();

    return false;
}

//////////////////////////////

bool CLR_RT_GenericParam_Instance::InitializeFromIndex(const CLR_RT_GenericParam_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();

    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        m_target = m_assm->GetGenericParam(GenericParam());

        return true;
    }

    m_data = 0;
    m_assm = NULL;
    m_target = NULL;

    return false;
}

void CLR_RT_GenericParam_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_GenericParam_Index::Clear();

    m_assm = NULL;
    m_target = NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CLR_RT_MethodSpec_Instance::InitializeFromIndex(const CLR_RT_MethodSpec_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();

    if (NANOCLR_INDEX_IS_VALID(index))
    {
        m_data = index.m_data;
        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        m_target = m_assm->GetMethodSpec(Method());

        return true;
    }

    m_data = 0;
    m_assm = NULL;
    m_target = NULL;

    return false;
}

void CLR_RT_MethodSpec_Instance::Clear()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_MethodSpec_Index::Clear();

    m_assm = NULL;
    m_target = NULL;
}

bool CLR_RT_MethodSpec_Instance::ResolveToken(CLR_UINT32 tk, CLR_RT_Assembly* assm)
{
    NATIVE_PROFILE_CLR_CORE();
    if (assm)
    {
        CLR_UINT32 index = CLR_DataFromTk(tk);

        //switch (CLR_TypeFromTk(tk))
        //{
        //    case TBL_MethodRef:
        //        m_data = assm->m_pCrossReference_MethodRef[index].m_target.m_data;
        //        m_assm = g_CLR_RT_TypeSystem.m_assemblies[Assembly() - 1];
        //        m_target = m_assm->GetMethodDef(Method());
        //        return true;

        //    case TBL_MethodDef:
        //        Set(assm->m_index, index);

        //        m_assm = assm;
        //        m_target = m_assm->GetMethodDef(index);
        //        return true;

        //    case TBL_MethodSpec:
        //    {
        //        Set(assm->m_index, index);

        //        m_assm = assm;


        //        //const CLR_RECORD_METHODSPEC* temp_target = m_assm->GetMethodSpec(index);

        //        m_target = 0;

        //        return true;
        //    }
        //    case TBL_TypeSpec:
        //    {
        //        CLR_RT_TypeSpec_Index typeSpec;
        //        typeSpec.Set(assm->m_index, index);

        //        CLR_RT_TypeSpec_Instance typeSpecInstance;
        //        typeSpecInstance.InitializeFromIndex(typeSpec);

        //        CLR_RT_MethodSpec_Index methodSpec;
        //        FindMethodSpec(typeSpecInstance.m_target, this, p->sig, methodSpec);


        //        CLR_RT_MethodDef_Index methodDef;
        //        FindMethodDef(typeSpecInstance.m_target, GetString(p->name), this, p->sig, methodDef);



        //        Set(assm->m_index, index);

        //        m_assm = assm;


        //        //const CLR_RECORD_METHODSPEC* temp_target = m_assm->GetMethodSpec(index);

        //        m_target = 0;

        //        return true;
        //    }
        //    default:
        //        // the remaining data types aren't to be handled
        //        break;
        //}
    }

    Clear();

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CLR_RT_TypeDescriptor::TypeDescriptor_Initialize()
{
    NATIVE_PROFILE_CLR_CORE();
    m_flags = 0;
    m_handlerCls.Clear();
    m_handlerGenericType.Clear();
    m_reflex.Clear();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromDataType(nanoClrDataType dt)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if (dt >= DATATYPE_FIRST_INVALID)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }
    else
    {
        const CLR_RT_DataTypeLookup &dtl = c_CLR_RT_DataTypeLookup[dt];

        m_flags = dtl.m_flags & CLR_RT_DataTypeLookup::c_SemanticMask2;

        if (dtl.m_cls)
        {
            if (m_handlerCls.InitializeFromIndex(*dtl.m_cls) == false)
            {
                NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
            }

            m_reflex.m_kind = REFLECTION_TYPE;
            m_reflex.m_levels = 0;
            m_reflex.m_data.m_type = *dtl.m_cls;
        }
        else
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
        }
    }

    m_handlerGenericType.Clear();

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromReflection(const CLR_RT_ReflectionDef_Index &reflex)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_TypeDef_Instance inst;
    CLR_UINT32 levels;

    if (inst.InitializeFromReflection(reflex, &levels) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    NANOCLR_CHECK_HRESULT(InitializeFromType(inst));

    if (levels)
    {
        m_reflex.m_levels = levels;

        ConvertToArray();
    }

    m_handlerCls.Clear();
    m_handlerGenericType.Clear();

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromTypeSpec(const CLR_RT_TypeSpec_Index &sig)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_TypeSpec_Instance inst;
    CLR_RT_SignatureParser parser;

    if (inst.InitializeFromIndex(sig) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    //parser.Initialize_TypeSpec(inst.m_assm, inst.m_target);

    NANOCLR_SET_AND_LEAVE(InitializeFromSignatureParser(parser));

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromType(const CLR_RT_TypeDef_Index &cls)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if (m_handlerCls.InitializeFromIndex(cls) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }
    else
    {
        const CLR_RT_DataTypeLookup &dtl = c_CLR_RT_DataTypeLookup[m_handlerCls.m_target->DataType];

        m_flags = dtl.m_flags & CLR_RT_DataTypeLookup::c_SemanticMask;

        m_reflex.m_kind = REFLECTION_TYPE;
        m_reflex.m_levels = 0;
        m_reflex.m_data.m_type = m_handlerCls;

        if (m_flags == CLR_RT_DataTypeLookup::c_Primitive)
        {
            if ((m_handlerCls.m_target->Flags & CLR_RECORD_TYPEDEF::TD_Semantics_Mask) ==
                CLR_RECORD_TYPEDEF::TD_Semantics_Enum)
            {
                m_flags = CLR_RT_DataTypeLookup::c_Enum;
            }
        }
        else
        {
            switch (m_handlerCls.m_target->Flags & CLR_RECORD_TYPEDEF::TD_Semantics_Mask)
            {
                case CLR_RECORD_TYPEDEF::TD_Semantics_ValueType:
                    m_flags = CLR_RT_DataTypeLookup::c_ValueType;
                    break;
                case CLR_RECORD_TYPEDEF::TD_Semantics_Class:
                    m_flags = CLR_RT_DataTypeLookup::c_Class;
                    break;
                case CLR_RECORD_TYPEDEF::TD_Semantics_Interface:
                    m_flags = CLR_RT_DataTypeLookup::c_Interface;
                    break;
                case CLR_RECORD_TYPEDEF::TD_Semantics_Enum:
                    m_flags = CLR_RT_DataTypeLookup::c_Enum;
                    break;
            }
        }

        if (m_handlerCls.m_data == g_CLR_RT_WellKnownTypes.m_Array.m_data)
        {
            m_flags |= CLR_RT_DataTypeLookup::c_Array;
        }

        if (m_handlerCls.m_data == g_CLR_RT_WellKnownTypes.m_ArrayList.m_data)
        {
            m_flags |= CLR_RT_DataTypeLookup::c_ArrayList;
        }
    }

    m_handlerGenericType.Clear();

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromGenericType(const CLR_RT_TypeSpec_Index& genericType)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if (m_handlerGenericType.InitializeFromIndex(genericType) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }
    else
    {
        m_flags = CLR_RT_DataTypeLookup::c_ManagedType | CLR_RT_DataTypeLookup::c_GenericInstance;

        m_reflex.m_kind = REFLECTION_GENERICTYPE;
        m_reflex.m_data.m_genericType = m_handlerGenericType;
    }
    
    m_handlerCls.Clear();

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromFieldDefinition(const CLR_RT_FieldDef_Instance &fd)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_SignatureParser parser;
    parser.Initialize_FieldDef(fd.m_assm, fd.m_target);

    NANOCLR_SET_AND_LEAVE(InitializeFromSignatureParser(parser));

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromSignatureParser(CLR_RT_SignatureParser& parser)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_SignatureParser::Element res;

    if (parser.Available() <= 0)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_OUT_OF_RANGE);
    }

    NANOCLR_CHECK_HRESULT(parser.Advance(res));

    if (res.DataType == DATATYPE_GENERICINST)
    {
        NANOCLR_CHECK_HRESULT(InitializeFromGenericType(res.TypeSpec));
    }
    else
    {
        NANOCLR_CHECK_HRESULT(InitializeFromType(res.Class));
    }
    
    if (res.Levels)
    {
        m_reflex.m_levels = res.Levels;

        ConvertToArray();
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::InitializeFromObject(const CLR_RT_HeapBlock &ref)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    const CLR_RT_HeapBlock *obj = &ref;
    nanoClrDataType dt;

    while (true)
    {
        dt = (nanoClrDataType)obj->DataType();

        if (dt == DATATYPE_BYREF || dt == DATATYPE_OBJECT)
        {
            obj = obj->Dereference();
            FAULT_ON_NULL(obj);
        }
#if defined(NANOCLR_APPDOMAINS)
        else if (dt == DATATYPE_TRANSPARENT_PROXY)
        {
            NANOCLR_CHECK_HRESULT(obj->TransparentProxyValidate());
            obj = obj->TransparentProxyDereference();
        }
#endif
        else
        {
            break;
        }
    }

    {
        const CLR_RT_TypeDef_Index *cls = NULL;
        const CLR_RT_ReflectionDef_Index *reflex = NULL;
        const CLR_RT_TypeSpec_Index* genericType = NULL;

        switch (dt)
        {
            case DATATYPE_SZARRAY:
                reflex = &obj->ReflectionDataConst();
                cls = &reflex->m_data.m_type;
                break;

            case DATATYPE_VALUETYPE:
            case DATATYPE_CLASS:
                cls = &obj->ObjectCls();
                break;

            case DATATYPE_DELEGATE_HEAD:
            {
                CLR_RT_HeapBlock_Delegate *dlg = (CLR_RT_HeapBlock_Delegate *)obj;

                cls = NANOCLR_INDEX_IS_VALID(dlg->m_cls) ? &dlg->m_cls : &g_CLR_RT_WellKnownTypes.m_Delegate;
            }
            break;

            case DATATYPE_DELEGATELIST_HEAD:
            {
                CLR_RT_HeapBlock_Delegate_List *dlgLst = (CLR_RT_HeapBlock_Delegate_List *)obj;

                cls = NANOCLR_INDEX_IS_VALID(dlgLst->m_cls) ? &dlgLst->m_cls
                                                            : &g_CLR_RT_WellKnownTypes.m_MulticastDelegate;
            }
            break;

                //--//

            case DATATYPE_WEAKCLASS:
            {
                cls = &g_CLR_RT_WellKnownTypes.m_WeakReference;
            }
            break;

                //--//

            case DATATYPE_REFLECTION:
                reflex = &(obj->ReflectionDataConst());

                switch (reflex->m_kind)
                {
                    case REFLECTION_ASSEMBLY:
                        cls = &g_CLR_RT_WellKnownTypes.m_Assembly;
                        break;
                    case REFLECTION_TYPE:
                        cls = &g_CLR_RT_WellKnownTypes.m_Type;
                        break;
                    case REFLECTION_TYPE_DELAYED:
                        cls = &g_CLR_RT_WellKnownTypes.m_Type;
                        break;
                    case REFLECTION_CONSTRUCTOR:
                        cls = &g_CLR_RT_WellKnownTypes.m_ConstructorInfo;
                        break;
                    case REFLECTION_METHOD:
                        cls = &g_CLR_RT_WellKnownTypes.m_MethodInfo;
                        break;
                    case REFLECTION_FIELD:
                        cls = &g_CLR_RT_WellKnownTypes.m_FieldInfo;
                        break;
                }

                break;

                //--//

            case DATATYPE_ARRAY_BYREF:
            {
                CLR_RT_HeapBlock_Array *array = obj->Array();
                FAULT_ON_NULL(array);

                if (array->m_fReference)
                {
                    obj = (CLR_RT_HeapBlock *)array->GetElement(obj->ArrayIndex());

                    NANOCLR_SET_AND_LEAVE(InitializeFromObject(*obj));
                }

                reflex = &array->ReflectionDataConst();
                cls = &reflex->m_data.m_type;
            }
            break;

            case DATATYPE_GENERICINST:
                genericType = &obj->ObjectGenericType();
                break;

                //--//

            default:
                NANOCLR_SET_AND_LEAVE(InitializeFromDataType(dt));
        }

        if (cls)
        {
            NANOCLR_CHECK_HRESULT(InitializeFromType(*cls));
        }

        if (reflex)
        {
            m_reflex = *reflex;
        }

        if (genericType)
        {
            NANOCLR_CHECK_HRESULT(InitializeFromGenericType(*genericType));
        }

        if (dt == DATATYPE_SZARRAY)
        {
            ConvertToArray();
        }
    }

    NANOCLR_NOCLEANUP();
}

////////////////////////////////////////

void CLR_RT_TypeDescriptor::ConvertToArray()
{
    NATIVE_PROFILE_CLR_CORE();
    m_flags &= CLR_RT_DataTypeLookup::c_SemanticMask;
    m_flags |= CLR_RT_DataTypeLookup::c_Array;

    m_handlerCls.InitializeFromIndex(g_CLR_RT_WellKnownTypes.m_Array);
}

bool CLR_RT_TypeDescriptor::ShouldEmitHash()
{
    NATIVE_PROFILE_CLR_CORE();
    if (m_flags & (CLR_RT_DataTypeLookup::c_Array | CLR_RT_DataTypeLookup::c_ArrayList))
    {
        return true;
    }

    if (m_flags &
        (CLR_RT_DataTypeLookup::c_Primitive | CLR_RT_DataTypeLookup::c_ValueType | CLR_RT_DataTypeLookup::c_Enum))
    {
        return false;
    }

    if (m_handlerCls.CrossReference().m_hash != 0)
    {
        return true;
    }

    return false;
}

bool CLR_RT_TypeDescriptor::GetElementType(CLR_RT_TypeDescriptor &sub)
{
    NATIVE_PROFILE_CLR_CORE();
    switch (m_reflex.m_levels)
    {
        case 0:
            return false;

        case 1:
            sub.InitializeFromType(m_reflex.m_data.m_type);
            break;

        default:
            sub = *this;
            sub.m_reflex.m_levels--;
            break;
    }

    return true;
}

////////////////////////////////////////

HRESULT CLR_RT_TypeDescriptor::ExtractObjectAndDataType(CLR_RT_HeapBlock *&ref, nanoClrDataType &dt)
{
    NATIVE_PROFILE_CLR_CORE();

    NANOCLR_HEADER();

    while (true)
    {
        dt = (nanoClrDataType)ref->DataType();

        if (dt == DATATYPE_BYREF || dt == DATATYPE_OBJECT)
        {
            ref = ref->Dereference();
            FAULT_ON_NULL(ref);
        }
        else
        {
            break;
        }
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeDescriptor::ExtractTypeIndexFromObject(const CLR_RT_HeapBlock &ref, CLR_RT_TypeDef_Index &res)
{
    NATIVE_PROFILE_CLR_CORE();

    NANOCLR_HEADER();

    CLR_RT_HeapBlock *obj = (CLR_RT_HeapBlock *)&ref;
    nanoClrDataType dt;

    NANOCLR_CHECK_HRESULT(CLR_RT_TypeDescriptor::ExtractObjectAndDataType(obj, dt));

    if (dt == DATATYPE_VALUETYPE || dt == DATATYPE_CLASS)
    {
        res = obj->ObjectCls();
    }
    else
    {
        const CLR_RT_DataTypeLookup &dtl = c_CLR_RT_DataTypeLookup[dt];

        if (dtl.m_cls)
        {
            res = *dtl.m_cls;
        }
        else
        {
            res.Clear();
        }
    }

    if (NANOCLR_INDEX_IS_INVALID(res))
    {
        CLR_RT_TypeDescriptor desc;

        NANOCLR_CHECK_HRESULT(desc.InitializeFromObject(ref))

        // If desc.InitializeFromObject( ref ) call succeed, then we need to find out what to call

        if (desc.GetDataType() == DATATYPE_GENERICINST)
        {
            res.Set(desc.m_handlerGenericType.Assembly(), desc.m_handlerGenericType.TypeDefIndex);
        }
        else
        {
            // use m_handlerCls for res
            res = desc.m_handlerCls;
        }

        if (NANOCLR_INDEX_IS_INVALID(res))
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
        }
    }

    NANOCLR_NOCLEANUP();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

// Keep these strings less than 8-character long (including terminator) because it's stuffed into an 8-byte structure.
// static const char c_MARKER_ASSEMBLY_V1[] = "NFMRK1";
static const char c_MARKER_ASSEMBLY_V2[] = "NFMRK2";

/// @brief Check for valid assembly header (CRC32 of header, string table version and marker)
///
/// @return Check result
bool CLR_RECORD_ASSEMBLY::GoodHeader() const
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RECORD_ASSEMBLY header = *this;
    header.headerCRC = 0;

    if (SUPPORT_ComputeCRC(&header, sizeof(header), 0) != this->headerCRC)
    {
        return false;
    }

    if (this->stringTableVersion != c_CLR_StringTable_Version)
    {
        return false;
    }

    return memcmp(marker, c_MARKER_ASSEMBLY_V2, sizeof(c_MARKER_ASSEMBLY_V2)) == 0;
}

/// @brief Check for valid assembly (header and CRC32 of assembly content)
///
/// @return bool Check result
bool CLR_RECORD_ASSEMBLY::GoodAssembly() const
{
    NATIVE_PROFILE_CLR_CORE();
    if (!GoodHeader())
    {
        return false;
    }

    return SUPPORT_ComputeCRC(&this[1], this->TotalSize() - sizeof(*this), 0) == this->assemblyCRC;
}

CLR_UINT32 CLR_RECORD_ASSEMBLY::ComputeAssemblyHash(const char *name, const CLR_RECORD_VERSION &ver)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_UINT32 assemblyHASH;

    assemblyHASH = SUPPORT_ComputeCRC(name, (int)hal_strlen_s(name), 0);
    assemblyHASH = SUPPORT_ComputeCRC(&ver, sizeof(ver), assemblyHASH);

    return assemblyHASH;
}

//--//

CLR_PMETADATA CLR_RECORD_EH::ExtractEhFromByteCode(CLR_PMETADATA ipEnd, const CLR_RECORD_EH *&ptrEh, CLR_UINT32 &numEh)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_UINT32 num = *(--ipEnd);
    ipEnd -= sizeof(CLR_RECORD_EH) * num;

    numEh = num;
    ptrEh = (const CLR_RECORD_EH *)ipEnd;

    return ipEnd;
}

CLR_UINT32 CLR_RECORD_EH::GetToken() const
{
    NATIVE_PROFILE_CLR_CORE();
    if (classToken & 0x8000)
    {
        return CLR_TkFromType(TBL_TypeRef, classToken & 0x7FFF);
    }
    else
    {
        return CLR_TkFromType(TBL_TypeDef, classToken);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CLR_RT_ExceptionHandler::ConvertFromEH(
    const CLR_RT_MethodDef_Instance &owner,
    CLR_PMETADATA ipStart,
    const CLR_RECORD_EH *ehPtr)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RECORD_EH eh;
    memcpy(&eh, ehPtr, sizeof(eh));

    switch (eh.mode)
    {
        case CLR_RECORD_EH::EH_Finally:
            m_typeFilter.Clear();
            break;

        case CLR_RECORD_EH::EH_Filter:
            m_userFilterStart = ipStart + eh.filterStart;
            break;

        case CLR_RECORD_EH::EH_CatchAll:
            m_typeFilter = g_CLR_RT_WellKnownTypes.m_Object;
            break;

        case CLR_RECORD_EH::EH_Catch:
        {
            CLR_RT_TypeDef_Instance cls;
            if (cls.ResolveToken(eh.GetToken(), owner.m_assm) == false)
                return false;
            m_typeFilter = cls;
        }
        break;

        default:
            return false;
    }

    if (owner.m_target->RVA == CLR_EmptyIndex)
        return false;

    m_ehType = eh.mode;
    m_tryStart = ipStart + eh.tryStart;
    m_tryEnd = ipStart + eh.tryEnd;
    m_handlerStart = ipStart + eh.handlerStart;
    m_handlerEnd = ipStart + eh.handlerEnd;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CLR_RT_Assembly::IsSameAssembly(const CLR_RT_Assembly &assm) const
{
    if (m_header->headerCRC == assm.m_header->headerCRC && m_header->assemblyCRC == assm.m_header->assemblyCRC)
    {
        return true;
    }

    return false;
}

void CLR_RT_Assembly::Assembly_Initialize(CLR_RT_Assembly::Offsets &offsets)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_UINT8 *buffer = (CLR_UINT8 *)this;
    int i;

    m_szName = GetString(m_header->assemblyName);

    //--//
    buffer += offsets.iBase;
    m_pCrossReference_AssemblyRef = (CLR_RT_AssemblyRef_CrossReference *)buffer;
    buffer += offsets.iAssemblyRef;
    m_pCrossReference_TypeRef = (CLR_RT_TypeRef_CrossReference *)buffer;
    buffer += offsets.iTypeRef;
    m_pCrossReference_FieldRef = (CLR_RT_FieldRef_CrossReference *)buffer;
    buffer += offsets.iFieldRef;
    m_pCrossReference_MethodRef = (CLR_RT_MethodRef_CrossReference *)buffer;
    buffer += offsets.iMethodRef;
    m_pCrossReference_TypeDef = (CLR_RT_TypeDef_CrossReference *)buffer;
    buffer += offsets.iTypeDef;
    m_pCrossReference_FieldDef = (CLR_RT_FieldDef_CrossReference *)buffer;
    buffer += offsets.iFieldDef;
    m_pCrossReference_MethodDef = (CLR_RT_MethodDef_CrossReference *)buffer;
    buffer += offsets.iMethodDef;
    m_pCrossReference_GenericParam = (CLR_RT_GenericParam_CrossReference *)buffer;
    buffer += offsets.iGenericParam;
    m_pCrossReference_MethodSpec = (CLR_RT_MethodSpec_CrossReference *)buffer;
    buffer += offsets.iMethodSpec;
    m_pCrossReference_TypeSpec = (CLR_RT_TypeSpec_CrossReference *)buffer;
    buffer += offsets.iTypeSpec;

#if !defined(NANOCLR_APPDOMAINS)
    m_pStaticFields = (CLR_RT_HeapBlock *)buffer;
    buffer += offsets.iStaticFields;

    memset(m_pStaticFields, 0, offsets.iStaticFields);
#endif

    //--//

    {
        const CLR_RECORD_TYPEDEF *src = (const CLR_RECORD_TYPEDEF *)this->GetTable(TBL_TypeDef);
        CLR_RT_TypeDef_CrossReference *dst = this->m_pCrossReference_TypeDef;
        for (i = 0; i < this->m_pTablesSize[TBL_TypeDef]; i++, src++, dst++)
        {
            dst->m_flags = 0;
            dst->m_totalFields = 0;
            dst->m_hash = 0;
        }
    }

    {
        const CLR_RECORD_FIELDDEF *src = (const CLR_RECORD_FIELDDEF *)this->GetTable(TBL_FieldDef);
        CLR_RT_FieldDef_CrossReference *dst = this->m_pCrossReference_FieldDef;
        for (i = 0; i < this->m_pTablesSize[TBL_FieldDef]; i++, src++, dst++)
        {
            dst->m_offset = CLR_EmptyIndex;
        }
    }

    {
        const CLR_RECORD_METHODDEF *src = (const CLR_RECORD_METHODDEF *)this->GetTable(TBL_MethodDef);
        CLR_RT_MethodDef_CrossReference *dst = this->m_pCrossReference_MethodDef;
        for (i = 0; i < this->m_pTablesSize[TBL_MethodDef]; i++, src++, dst++)
        {
            dst->m_data = CLR_EmptyIndex;
        }
    }

    {
        const CLR_RECORD_GENERICPARAM *src = (const CLR_RECORD_GENERICPARAM *)this->GetTable(TBL_GenericParam);
        CLR_RT_GenericParam_CrossReference *dst = this->m_pCrossReference_GenericParam;
        for (i = 0; i < this->m_pTablesSize[TBL_GenericParam]; i++, src++, dst++)
        {
            dst->m_data = CLR_EmptyIndex;
        }
    }

    {
        const CLR_RECORD_METHODSPEC *src = (const CLR_RECORD_METHODSPEC *)this->GetTable(TBL_MethodSpec);
        CLR_RT_MethodSpec_CrossReference *dst = this->m_pCrossReference_MethodSpec;
        for (i = 0; i < this->m_pTablesSize[TBL_MethodSpec]; i++, src++, dst++)
        {
            dst->m_data = CLR_EmptyIndex;
        }
    }

    {
        const CLR_RECORD_TYPESPEC* src = (const CLR_RECORD_TYPESPEC*)this->GetTable(TBL_TypeSpec);
        CLR_RT_TypeSpec_CrossReference* dst = this->m_pCrossReference_TypeSpec;
        for (i = 0; i < this->m_pTablesSize[TBL_TypeSpec]; i++, src++, dst++)
        {
            dst->GenericType.m_data = 0;
        }
    }

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
    {
        m_pDebuggingInfo_MethodDef = (CLR_RT_MethodDef_DebuggingInfo *)buffer;
        buffer += offsets.iDebuggingInfoMethods;

        memset(m_pDebuggingInfo_MethodDef, 0, offsets.iDebuggingInfoMethods);
    }
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
}

HRESULT CLR_RT_Assembly::CreateInstance(const CLR_RECORD_ASSEMBLY *header, CLR_RT_Assembly *&assm)
{
    NATIVE_PROFILE_CLR_CORE();
    //
    // We have to use this trick, otherwise the C++ compiler will try to all the constructor for Assembly.
    //
    NANOCLR_HEADER();

    CLR_UINT8 buf[sizeof(CLR_RT_Assembly)];
    CLR_RT_Assembly *skeleton = (CLR_RT_Assembly *)buf;

    NANOCLR_CLEAR(*skeleton);

    if (header->GoodAssembly() == false)
    {
        NANOCLR_MSG_SET_AND_LEAVE(CLR_E_FAIL, L"Failed in type system: assembly is not good.\n");
    }

    skeleton->m_header = header;

    // Compute overall size for assembly data structure.
    {
        for (uint32_t i = 0; i < ARRAYSIZE(skeleton->m_pTablesSize) - 1; i++)
        {
            skeleton->m_pTablesSize[i] = header->SizeOfTable((nanoClrTable)i);
        }

        skeleton->m_pTablesSize[TBL_AssemblyRef] /= sizeof(CLR_RECORD_ASSEMBLYREF);
        skeleton->m_pTablesSize[TBL_TypeRef] /= sizeof(CLR_RECORD_TYPEREF);
        skeleton->m_pTablesSize[TBL_FieldRef] /= sizeof(CLR_RECORD_FIELDREF);
        skeleton->m_pTablesSize[TBL_MethodRef] /= sizeof(CLR_RECORD_METHODREF);
        skeleton->m_pTablesSize[TBL_TypeDef] /= sizeof(CLR_RECORD_TYPEDEF);
        skeleton->m_pTablesSize[TBL_FieldDef] /= sizeof(CLR_RECORD_FIELDDEF);
        skeleton->m_pTablesSize[TBL_MethodDef] /= sizeof(CLR_RECORD_METHODDEF);
        skeleton->m_pTablesSize[TBL_GenericParam] /= sizeof(CLR_RECORD_GENERICPARAM);
        skeleton->m_pTablesSize[TBL_MethodSpec] /= sizeof(CLR_RECORD_METHODSPEC);
        skeleton->m_pTablesSize[TBL_TypeSpec] /= sizeof(CLR_RECORD_TYPESPEC);
        skeleton->m_pTablesSize[TBL_Attributes] /= sizeof(CLR_RECORD_ATTRIBUTE);
        skeleton->m_pTablesSize[TBL_Resources] /= sizeof(CLR_RECORD_RESOURCE);
        skeleton->m_pTablesSize[TBL_ResourcesFiles] /= sizeof(CLR_RECORD_RESOURCE_FILE);
    }

    //--//

    // Count static fields.
    {
        const CLR_RECORD_TYPEDEF *src = (const CLR_RECORD_TYPEDEF *)skeleton->GetTable(TBL_TypeDef);

        for (int i = 0; i < skeleton->m_pTablesSize[TBL_TypeDef]; i++, src++)
        {
            skeleton->m_iStaticFields += src->StaticFieldsCount;
        }
    }

    //--//

    {
        CLR_RT_Assembly::Offsets offsets;

        offsets.iBase = ROUNDTOMULTIPLE(sizeof(CLR_RT_Assembly), CLR_UINT32);

        offsets.iAssemblyRef = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_AssemblyRef] * sizeof(CLR_RT_AssemblyRef_CrossReference),
            CLR_UINT32);

        offsets.iTypeRef =
            ROUNDTOMULTIPLE(skeleton->m_pTablesSize[TBL_TypeRef] * sizeof(CLR_RT_TypeRef_CrossReference), CLR_UINT32);

        offsets.iFieldRef =
            ROUNDTOMULTIPLE(skeleton->m_pTablesSize[TBL_FieldRef] * sizeof(CLR_RT_FieldRef_CrossReference), CLR_UINT32);

        offsets.iMethodRef = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_MethodRef] * sizeof(CLR_RT_MethodRef_CrossReference),
            CLR_UINT32);

        offsets.iTypeDef =
            ROUNDTOMULTIPLE(skeleton->m_pTablesSize[TBL_TypeDef] * sizeof(CLR_RT_TypeDef_CrossReference), CLR_UINT32);

        offsets.iFieldDef =
            ROUNDTOMULTIPLE(skeleton->m_pTablesSize[TBL_FieldDef] * sizeof(CLR_RT_FieldDef_CrossReference), CLR_UINT32);

        offsets.iMethodDef = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_MethodDef] * sizeof(CLR_RT_MethodDef_CrossReference),
            CLR_UINT32);

        offsets.iGenericParam = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_GenericParam] * sizeof(CLR_RT_GenericParam_CrossReference),
            CLR_UINT32);

        offsets.iMethodSpec = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_MethodSpec] * sizeof(CLR_RT_MethodSpec_CrossReference),
            CLR_UINT32);

        offsets.iTypeSpec = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_TypeSpec] * sizeof(CLR_RT_TypeSpec_CrossReference),
            CLR_UINT32);

#if !defined(NANOCLR_APPDOMAINS)
        offsets.iStaticFields = ROUNDTOMULTIPLE(skeleton->m_iStaticFields * sizeof(CLR_RT_HeapBlock), CLR_UINT32);
#endif

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
        offsets.iDebuggingInfoMethods = ROUNDTOMULTIPLE(
            skeleton->m_pTablesSize[TBL_MethodDef] * sizeof(CLR_RT_MethodDef_DebuggingInfo),
            CLR_UINT32);
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)

        size_t iTotalRamSize = offsets.iBase + offsets.iAssemblyRef + offsets.iTypeRef + offsets.iFieldRef +
                               offsets.iMethodRef + offsets.iTypeDef + offsets.iFieldDef + offsets.iMethodDef + 
                               offsets.iGenericParam + offsets.iMethodSpec + offsets.iTypeSpec;

#if !defined(NANOCLR_APPDOMAINS)
        iTotalRamSize += offsets.iStaticFields;
#endif

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
        iTotalRamSize += offsets.iDebuggingInfoMethods;
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)

        //--//

        assm = EVENTCACHE_EXTRACT_NODE_AS_BYTES(
            g_CLR_RT_EventCache,
            CLR_RT_Assembly,
            DATATYPE_ASSEMBLY,
            0,
            (CLR_UINT32)iTotalRamSize);
        CHECK_ALLOCATION(assm);

        {
            //
            // We don't want to blow away the block header...
            //
            CLR_RT_HeapBlock *src = skeleton;
            CLR_RT_HeapBlock *dst = assm;

            memset(&dst[1], 0, iTotalRamSize - sizeof(CLR_RT_HeapBlock));
            memcpy(&dst[1], &src[1], sizeof(*assm) - sizeof(CLR_RT_HeapBlock));
        }

        assm->Assembly_Initialize(offsets);

#if !defined(BUILD_RTM)
        CLR_Debug::Printf(
            "   Assembly: %s (%d.%d.%d.%d)  ",
            assm->m_szName,
            header->version.iMajorVersion,
            header->version.iMinorVersion,
            header->version.iBuildNumber,
            header->version.iRevisionNumber);

        if (s_CLR_RT_fTrace_AssemblyOverhead >= c_CLR_RT_Trace_Info)
        {
            size_t iMetaData = header->SizeOfTable(TBL_AssemblyRef) + header->SizeOfTable(TBL_TypeRef) +
                               header->SizeOfTable(TBL_FieldRef) + header->SizeOfTable(TBL_MethodRef) +
                               header->SizeOfTable(TBL_TypeDef) + header->SizeOfTable(TBL_FieldDef) +
                               header->SizeOfTable(TBL_MethodDef) + header->SizeOfTable(TBL_GenericParam) +
                               header->SizeOfTable(TBL_MethodSpec) + header->SizeOfTable(TBL_TypeSpec) +
                header->SizeOfTable(TBL_Attributes) +
                                header->SizeOfTable(TBL_Signatures);

            CLR_Debug::Printf(
                " (%d RAM - %d ROM - %d METADATA)\r\n\r\n",
                iTotalRamSize,
                header->TotalSize(),
                iMetaData);

            CLR_Debug::Printf(
                "   AssemblyRef     = %6d bytes (%5d elements)\r\n",
                offsets.iAssemblyRef,
                skeleton->m_pTablesSize[TBL_AssemblyRef]);
            CLR_Debug::Printf(
                "   TypeRef         = %6d bytes (%5d elements)\r\n",
                offsets.iTypeRef,
                skeleton->m_pTablesSize[TBL_TypeRef]);
            CLR_Debug::Printf(
                "   FieldRef        = %6d bytes (%5d elements)\r\n",
                offsets.iFieldRef,
                skeleton->m_pTablesSize[TBL_FieldRef]);
            CLR_Debug::Printf(
                "   MethodRef       = %6d bytes (%5d elements)\r\n",
                offsets.iMethodRef,
                skeleton->m_pTablesSize[TBL_MethodRef]);
            CLR_Debug::Printf(
                "   TypeDef         = %6d bytes (%5d elements)\r\n",
                offsets.iTypeDef,
                skeleton->m_pTablesSize[TBL_TypeDef]);
            CLR_Debug::Printf(
                "   FieldDef        = %6d bytes (%5d elements)\r\n",
                offsets.iFieldDef,
                skeleton->m_pTablesSize[TBL_FieldDef]);
            CLR_Debug::Printf(
                "   MethodDef       = %6d bytes (%5d elements)\r\n",
                offsets.iMethodDef,
                skeleton->m_pTablesSize[TBL_MethodDef]);
            CLR_Debug::Printf(
                "   GenericParam    = %6d bytes (%5d elements)\r\n",
                offsets.iGenericParam,
                skeleton->m_pTablesSize[TBL_GenericParam]);
            CLR_Debug::Printf(
                "   MethodSpec      = %6d bytes (%5d elements)\r\n",
                offsets.iMethodSpec,
                skeleton->m_pTablesSize[TBL_MethodSpec]);

#if !defined(NANOCLR_APPDOMAINS)
            CLR_Debug::Printf(
                "   StaticFields    = %6d bytes (%5d elements)\r\n",
                offsets.iStaticFields,
                skeleton->m_iStaticFields);
#endif
            CLR_Debug::Printf("\r\n");

            CLR_Debug::Printf(
                "   Attributes      = %6d bytes (%5d elements)\r\n",
                skeleton->m_pTablesSize[TBL_Attributes] * sizeof(CLR_RECORD_ATTRIBUTE),
                skeleton->m_pTablesSize[TBL_Attributes]);
            CLR_Debug::Printf(
                "   TypeSpec        = %6d bytes (%5d elements)\r\n",
                skeleton->m_pTablesSize[TBL_TypeSpec] * sizeof(CLR_RECORD_TYPESPEC),
                skeleton->m_pTablesSize[TBL_TypeSpec]);
            CLR_Debug::Printf(
                "   Resources       = %6d bytes (%5d elements)\r\n",
                skeleton->m_pTablesSize[TBL_Resources] * sizeof(CLR_RECORD_RESOURCE),
                skeleton->m_pTablesSize[TBL_Resources]);
            CLR_Debug::Printf(
                "   Resources Files = %6d bytes (%5d elements)\r\n",
                skeleton->m_pTablesSize[TBL_ResourcesFiles] * sizeof(CLR_RECORD_RESOURCE),
                skeleton->m_pTablesSize[TBL_ResourcesFiles]);
            CLR_Debug::Printf("   Resources Data  = %6d bytes\r\n", skeleton->m_pTablesSize[TBL_ResourcesData]);
            CLR_Debug::Printf("   Strings         = %6d bytes\r\n", skeleton->m_pTablesSize[TBL_Strings]);
            CLR_Debug::Printf("   Signatures      = %6d bytes\r\n", skeleton->m_pTablesSize[TBL_Signatures]);
            CLR_Debug::Printf("   ByteCode        = %6d bytes\r\n", skeleton->m_pTablesSize[TBL_ByteCode]);
            CLR_Debug::Printf("\r\n\r\n");
        }
#endif
    }

    NANOCLR_NOCLEANUP();
}

#if defined(WIN32)
HRESULT CLR_RT_Assembly::CreateInstance(
    const CLR_RECORD_ASSEMBLY *header,
    CLR_RT_Assembly *&assm,
    const wchar_t *szName)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    std::string strPath;

    NANOCLR_CHECK_HRESULT(CLR_RT_Assembly::CreateInstance(header, assm));

    if (szName != NULL)
    {
        CLR_RT_UnicodeHelper::ConvertToUTF8(szName, strPath);

        assm->m_strPath = new std::string(strPath);
    }

    NANOCLR_NOCLEANUP();
}
#endif

bool CLR_RT_Assembly::Resolve_AssemblyRef(bool fOutput)
{
    NATIVE_PROFILE_CLR_CORE();
    bool fGot = true;
    int i;

    ITERATE_THROUGH_RECORDS(this, i, AssemblyRef, ASSEMBLYREF)
    {
        const char *szName = GetString(src->name);

        if (dst->m_target == NULL)
        {
            CLR_RT_Assembly *target = g_CLR_RT_TypeSystem.FindAssembly(szName, &src->version, false);

            if (target == NULL || (target->m_flags & CLR_RT_Assembly::Resolved) == 0)
            {
#if !defined(BUILD_RTM)
                if (fOutput)
                {
                    CLR_Debug::Printf(
                        "Assembly: %s (%d.%d.%d.%d)",
                        m_szName,
                        m_header->version.iMajorVersion,
                        m_header->version.iMinorVersion,
                        m_header->version.iBuildNumber,
                        m_header->version.iRevisionNumber);

                    CLR_Debug::Printf(
                        " needs assembly '%s' (%d.%d.%d.%d)\r\n",
                        szName,
                        src->version.iMajorVersion,
                        src->version.iMinorVersion,
                        src->version.iBuildNumber,
                        src->version.iRevisionNumber);
                }
#endif

                fGot = false;
            }
            else
            {
                dst->m_target = target;
            }
        }
    }

    return fGot;
}

void CLR_RT_Assembly::DestroyInstance()
{
    NATIVE_PROFILE_CLR_CORE();
    if (m_index)
    {
        g_CLR_RT_TypeSystem.m_assemblies[m_index - 1] = NULL;
    }

#if defined(WIN32)
    if (this->m_strPath != NULL)
    {
        delete this->m_strPath;
        this->m_strPath = NULL;
    }
#endif

    //--//

    g_CLR_RT_EventCache.Append_Node(this);
}

//--//
HRESULT CLR_RT_Assembly::Resolve_TypeRef()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    int i;

    ITERATE_THROUGH_RECORDS(this, i, TypeRef, TYPEREF)
    {
        // TODO check typedef
        if (src->Scope & 0x8000) // Flag for TypeRef
        {
            CLR_RT_TypeDef_Instance inst;

            if (inst.InitializeFromIndex(m_pCrossReference_TypeRef[src->Scope & 0x7FFF].m_target) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Resolve: unknown scope: %08x\r\n", src->Scope);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }

            const char *szName = GetString(src->Name);
            if (inst.m_assm->FindTypeDef(szName, inst.Type(), dst->m_target) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Resolve: unknown type: %s\r\n", szName);
#endif
                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }
        }
        else
        {
            CLR_RT_Assembly *assm = m_pCrossReference_AssemblyRef[src->Scope].m_target;
            if (assm == NULL)
            {
                NANOCLR_MSG_SET_AND_LEAVE(CLR_E_FAIL, L"Resolve: assm is null\n");
            }

            const char *szNameSpace = GetString(src->NameSpace);
            const char *szName = GetString(src->Name);
            if (assm->FindTypeDef(szName, szNameSpace, dst->m_target) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Resolve: unknown type: %s.%s\r\n", szNameSpace, szName);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }
        }
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_Assembly::Resolve_FieldRef()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    int i;

    ITERATE_THROUGH_RECORDS(this, i, FieldRef, FIELDREF)
    {
        CLR_RT_TypeDef_Index typeDef;
        typeDef.Clear();

        CLR_RT_TypeDef_Instance typeDefInstance;

        CLR_RT_TypeSpec_Index typeSpec;
        typeSpec.Clear();

        CLR_RT_TypeSpec_Instance typeSpecInstance;

        switch (src->Owner())
        {
            case TBL_TypeRef:
                typeDef = m_pCrossReference_TypeRef[src->OwnerIndex()].m_target;
                break;

                //case CLR_MemberRefParent::MRP_TypeDef:
                //    typeDef.Set(this->m_index, CLR_GetIndexFromMemberRefParent(src->container));
                //    break;
                //
                //case CLR_MemberRefParent::MRP_MethodDef:
                //    dst->m_target.Set(this->m_index, CLR_GetIndexFromMemberRefParent(src->container));
                //    fGot = true;
                //    break;

            case TBL_TypeSpec:
                typeSpec.Set(this->m_index, src->OwnerIndex());
                break;

            default:
#if !defined(BUILD_RTM)
                CLR_Debug::Printf(
                    "Unknown or unsupported TypeRefOrSpec when resolving FieldRef %08x\r\n",
                    src->encodedOwner);
#endif
                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
        }

        const char* fieldName = GetString(src->Name);

        if (NANOCLR_INDEX_IS_VALID(typeSpec))
        {
            if (typeSpecInstance.InitializeFromIndex(typeSpec) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Unknown scope when resolving FieldRef: %08x\r\n", src->encodedOwner);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }

            if (!FindFieldDef(typeSpecInstance.m_target, fieldName, this, src->Sig, dst->m_target))
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf(
                    "Unknown FieldRef: %s.%s.%s\r\n",
                    "???",
                    "???",
                    fieldName);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }
        }
        else if (NANOCLR_INDEX_IS_VALID(typeDef))
        {
            if (typeDefInstance.InitializeFromIndex(m_pCrossReference_TypeRef[src->OwnerIndex()].m_target) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Unknown scope when resolving FieldRef: %08x\r\n", src->encodedOwner);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }

#if defined(_WIN32) && defined(DEBUG_RESOLUTION)
            const CLR_RECORD_TYPEDEF* qTD = typeDefInstance.m_target;
            CLR_RT_Assembly* qASSM = typeDefInstance.m_assm;

            CLR_Debug::Printf(
                "Unknown scope when resolving FieldRef: %s.%s.%s\r\n",
                qASSM->GetString(qTD->NameSpace),
                qASSM->GetString(qTD->Name),
                name);
#endif

            if (typeDefInstance.m_assm->FindFieldDef(typeDefInstance.m_target, fieldName, this, src->Sig, dst->m_target) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Unknown FieldRef: %s\r\n", fieldName);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }
        }
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_Assembly::Resolve_MethodRef()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    int i;

    ITERATE_THROUGH_RECORDS(this, i, MethodRef, METHODREF)
    {
        CLR_RT_TypeDef_Index typeDef;
        typeDef.Clear();

        CLR_RT_TypeDef_Instance typeDefInstance;
       
        CLR_RT_TypeSpec_Index typeSpec;
        typeSpec.Clear(); 

        CLR_RT_TypeSpec_Instance typeSpecInstance;
        
        bool fGot = false;
        const char* name = NULL;

        switch (src->Owner())
        {
            case TBL_TypeRef:
                typeDef = m_pCrossReference_TypeRef[src->OwnerIndex()].m_target;
                break;

            //case CLR_MemberRefParent::MRP_TypeDef:
            //    typeDef.Set(this->m_index, CLR_GetIndexFromMemberRefParent(src->container));
            //    break;
            //
            //case CLR_MemberRefParent::MRP_MethodDef:
            //    dst->m_target.Set(this->m_index, CLR_GetIndexFromMemberRefParent(src->container));
            //    fGot = true;
            //    break;

            case TBL_TypeSpec:
                typeSpec.Set(this->m_index, src->OwnerIndex());
                break;

            default:
#if !defined(BUILD_RTM)
                CLR_Debug::Printf(
                    "Unknown or unsupported TypeRefOrSpec when resolving NethodRef %08x\r\n",
                    src->encodedOwner);
#endif
                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
        }

        name = GetString(src->Name);

        if (NANOCLR_INDEX_IS_VALID(typeSpec))
        {
            if (typeSpecInstance.InitializeFromIndex(typeSpec) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Unknown scope when resolving MethodRef: %08x\r\n", src->encodedOwner);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }

            if (FindMethodDef(typeSpecInstance.m_target, name, this, src->Sig, dst->Target))
            {
                fGot = true;

                // set TypeSpec
                dst->GenericType.m_data = typeSpec.m_data;

                // invalidate Target
                dst->Target.m_data = CLR_EmptyToken;
            }

            if (fGot == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf(
                    "Unknown MethodRef: %s.%s.%s\r\n",
                    "???",
                    "???",
                    name);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }
        }
        else if (NANOCLR_INDEX_IS_VALID(typeDef))
        {
            if (typeDefInstance.InitializeFromIndex(typeDef) == false)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Unknown scope when resolving MethodRef: %08x\r\n", src->encodedOwner);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }

#if defined(_WIN32) && defined(DEBUG_RESOLUTION)
            const CLR_RECORD_TYPEDEF* qTD = typeDefInstance.m_target;
            CLR_RT_Assembly* qASSM = typeDefInstance.m_assm;

            CLR_Debug::Printf(
                "Unknown scope when resolving MethodRef: %s.%s.%s\r\n",
                qASSM->GetString(qTD->NameSpace),
                qASSM->GetString(qTD->Name),
                name);
#endif

            while (NANOCLR_INDEX_IS_VALID(typeDefInstance))
            {
                if (typeDefInstance.m_assm->FindMethodDef(typeDefInstance.m_target, name, this, src->Sig, dst->Target))
                {
                    fGot = true;

                    // invalidate GenericType
                    dst->GenericType.m_data = CLR_EmptyToken;

                    break;
                }

                typeDefInstance.SwitchToParent();
            }

            if (fGot == false)
            {
#if !defined(BUILD_RTM)
                const CLR_RECORD_TYPEDEF* qTD = typeDefInstance.m_target;
                CLR_RT_Assembly* qASSM = typeDefInstance.m_assm;

                CLR_Debug::Printf(
                    "Unknown MethodRef: %s.%s.%s\r\n",
                    qASSM->GetString(qTD->NameSpace),
                    qASSM->GetString(qTD->Name),
                    name);
#endif

                NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
            }
        }
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_Assembly::Resolve_TypeSpec()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    int i;

    ITERATE_THROUGH_RECORDS(this, i, TypeSpec, TYPESPEC)
    {
        //CLR_RT_TypeSpec_Instance inst;

//        if (inst.InitializeFromIndex(m_pCrossReference_TypeSpec[src->sig].Signature) == false)
//        {
//#if !defined(BUILD_RTM)
//            CLR_Debug::Printf("Resolve TypeSpec: unknown signature: %08x\r\n", src->sig);
//#endif
//            NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
//        }
    }

    NANOCLR_NOCLEANUP_NOLABEL();
}

void CLR_RT_Assembly::Resolve_Link()
{
    NATIVE_PROFILE_CLR_CORE();
    int iStaticFields = 0;
    int indexType;

    ITERATE_THROUGH_RECORDS(this, indexType, TypeDef, TYPEDEF)
    {
        int num;
        int i;

        //
        // Link static fields.
        //
        {
            CLR_RT_FieldDef_CrossReference *fd = &m_pCrossReference_FieldDef[src->FirstStaticField];

            num = src->StaticFieldsCount;

            for (; num; num--, fd++)
            {
                fd->m_offset = iStaticFields++;
            }
        }

        //
        // Link instance fields.
        //
        {
            CLR_RT_TypeDef_Index index;
            index.Set(m_index, indexType);
            CLR_RT_TypeDef_Instance inst;
            inst.InitializeFromIndex(index);
            CLR_INDEX tot = 0;

            do
            {
                if (inst.m_target->Flags & CLR_RECORD_TYPEDEF::TD_HasFinalizer)
                {
                    dst->m_flags |= CLR_RT_TypeDef_CrossReference::TD_CR_HasFinalizer;
                }

#if defined(NANOCLR_APPDOMAINS)
                if (inst.m_data == g_CLR_RT_WellKnownTypes.m_MarshalByRefObject.m_data)
                {
                    dst->m_flags |= CLR_RT_TypeDef_CrossReference::TD_CR_IsMarshalByRefObject;
                }
#endif

                tot += inst.m_target->InstanceFieldsCount;
            } while (inst.SwitchToParent());

            dst->m_totalFields = tot;

            //--//

            CLR_RT_FieldDef_CrossReference *fd = &m_pCrossReference_FieldDef[src->FirstInstanceField];

            num = src->InstanceFieldsCount;
            i = tot - num + CLR_RT_HeapBlock::HB_Object_Fields_Offset; // Take into account the offset from the
                                                                       // beginning of the object.

            for (; num; num--, i++, fd++)
            {
                fd->m_offset = i;
            }
        }

        //
        // Link methods.
        //
        {
            CLR_RT_MethodDef_CrossReference *md = &m_pCrossReference_MethodDef[src->FirstMethod];

            int num = src->VirtualMethodCount + src->InstanceMethodCount + src->StaticMethodCount;

            for (; num; num--, md++)
            {
                md->m_data = indexType;
            }
        }

        //
        // Link generic parameters, if any
        //
        {
            if (src->GenericParamCount)
            {
                CLR_RT_GenericParam_CrossReference* gp = &m_pCrossReference_GenericParam[src->FirstGenericParam];

                // get generic parameter count for stop condition
                int num = src->GenericParamCount;
                CLR_UINT16 indexGenericParam = src->FirstGenericParam;

                for (; num; num--, gp++, indexGenericParam++)
                {
                    CLR_RT_GenericParam_Index gpIndex;
                    gpIndex.Set(m_index, indexGenericParam);

                    gp->m_target = gpIndex;

                    gp->m_data = indexType;
                    gp->m_TypeOrMethodDef = TBL_TypeDef;

                    CLR_RT_SignatureParser sub;
                    if (sub.Initialize_GenericParamTypeSignature(this, GetGenericParam(indexGenericParam)))
                    {
                        CLR_RT_SignatureParser::Element res;

                        // get generic param type
                        sub.Advance(res);

                        gp->DataType = res.DataType;
                        gp->Class = res.Class;
                    }
                    else
                    {
                        gp->DataType = DATATYPE_VOID;

                        CLR_RT_TypeDef_Index td;
                        td.Clear();

                        gp->Class = td;
                    }
                }
            }
        }
    }
}

//--//

#if defined(NANOCLR_APPDOMAINS)

HRESULT CLR_RT_AppDomain::CreateInstance(const char *szName, CLR_RT_AppDomain *&appDomain)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();
    CLR_RT_HeapBlock name;
    name.SetObjectReference(NULL);
    CLR_RT_ProtectFromGC gc(name);

    if (!szName || szName[0] == '\0')
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);

    // Check to see that the name does not already exist.
    NANOCLR_FOREACH_NODE(CLR_RT_AppDomain, appDomain, g_CLR_RT_ExecutionEngine.m_appDomains)
    {
        if (!strcmp(appDomain->m_strName->StringText(), szName))
            NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
    }
    NANOCLR_FOREACH_NODE_END();

    NANOCLR_CHECK_HRESULT(CLR_RT_HeapBlock_String::CreateInstance(name, szName));

    appDomain = EVENTCACHE_EXTRACT_NODE(g_CLR_RT_EventCache, CLR_RT_AppDomain, DATATYPE_APPDOMAIN_HEAD);
    CHECK_ALLOCATION(appDomain);

    appDomain->AppDomain_Initialize();

    appDomain->m_strName = name.DereferenceString();

    g_CLR_RT_ExecutionEngine.m_appDomains.LinkAtBack(appDomain);

    NANOCLR_NOCLEANUP();
}

void CLR_RT_AppDomain::RecoverFromGC()
{
    NATIVE_PROFILE_CLR_CORE();
    CheckAll();

    /*
        AppDomains can be zombied and stay around forever.  It is worth looking into cleaning up dead AppDomains, but
       this needs to be done with great care.  First, enumerations of AppDomains must not be allowed.  Second, the
       AppDomain must really be dead. This is much harder to ensure.  Everything that is needed to be checked for
       explicit AD unloading (stack frames, finalizable objects) must be checked, but also other things that can cause
       the AppDomain to be alive, like timers, manaaged drivers, transparent proxies, etc..
    */

    if (m_state == CLR_RT_AppDomain::AppDomainState_Unloaded)
    {
        // We could actually clean up here.  Since the AppDomain is now officially loaded,
        // we can remove all object_to_event_dst references, and null out the
        // pointers in the managed AppDomain class, provided that calling any method on the
        // AppDomain will result in a AppDomainDisposed exception.  However, it's probably
        // not worth the effort, the majority of the resources have been already cleaned
        // up, from AppDomain_Uninitialize

        if (IsReadyForRelease())
        {
            DestroyInstance();
        }
    }
}

void CLR_RT_AppDomain::AppDomain_Initialize()
{
    NATIVE_PROFILE_CLR_CORE();
    Initialize();

    m_appDomainAssemblies.DblLinkedList_Initialize();

    m_state = CLR_RT_AppDomain::AppDomainState_Loaded;
    m_id = g_CLR_RT_ExecutionEngine.m_appDomainIdNext++;
    m_globalLock = NULL;
    m_strName = NULL;
    m_outOfMemoryException = NULL;
    m_appDomainAssemblyLastAccess = NULL;
}

void CLR_RT_AppDomain::AppDomain_Uninitialize()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_FOREACH_NODE(CLR_RT_AppDomainAssembly, appDomainAssembly, m_appDomainAssemblies)
    {
        appDomainAssembly->DestroyInstance();
    }
    NANOCLR_FOREACH_NODE_END();
}

void CLR_RT_AppDomain::DestroyInstance()
{
    NATIVE_PROFILE_CLR_CORE();
    AppDomain_Uninitialize();

    Unlink();

    g_CLR_RT_EventCache.Append_Node(this);
}

HRESULT CLR_RT_AppDomain::LoadAssembly(CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_AppDomain *appDomainSav = g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(this);
    CLR_RT_AppDomainAssembly *appDomainAssembly = NULL;
    int i;

    FAULT_ON_NULL(assm);

    // check to make sure the assembly is not already loaded
    if (FindAppDomainAssembly(assm) != NULL)
        NANOCLR_SET_AND_LEAVE(S_OK);

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
    g_CLR_RT_ExecutionEngine.Breakpoint_Assemblies_Loaded();
#endif

    // Next, make sure that all dependent assemblies are loaded.
    {
        ITERATE_THROUGH_RECORDS(assm, i, AssemblyRef, ASSEMBLYREF)
        {
            NANOCLR_CHECK_HRESULT(LoadAssembly(dst->m_target));
        }
    }

    NANOCLR_CHECK_HRESULT(CLR_RT_AppDomainAssembly::CreateInstance(this, assm, appDomainAssembly));

    if (m_outOfMemoryException == NULL)
    {
        // Allocate an out of memory exception.  We should never get into a case where an out of memory exception
        // cannot be thrown.
        CLR_RT_HeapBlock exception;

        _ASSERTE(!strcmp(assm->m_szName, "mscorlib")); // always the first assembly to be loaded

        NANOCLR_CHECK_HRESULT(
            g_CLR_RT_ExecutionEngine.NewObjectFromIndex(exception, g_CLR_RT_WellKnownTypes.m_OutOfMemoryException));

        m_outOfMemoryException = exception.Dereference();
    }

    NANOCLR_CLEANUP();

    if (FAILED(hr))
    {
        if (appDomainAssembly)
        {
            appDomainAssembly->DestroyInstance();
        }
    }

    (void)g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomainSav);

    NANOCLR_CLEANUP_END();
}

HRESULT CLR_RT_AppDomain::GetManagedObject(CLR_RT_HeapBlock &res)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_AppDomain *appDomainSav = g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(this);

    res.SetObjectReference(NULL);

    // Check if a managed object is already present, and use it
    NANOCLR_FOREACH_NODE(CLR_RT_ObjectToEvent_Source, ref, m_references)
    {
        CLR_RT_HeapBlock *obj = ref->m_objectPtr;

        _ASSERTE(FIMPLIES(obj, obj->DataType() == DATATYPE_CLASS || obj->DataType() == DATATYPE_VALUETYPE));

        if (obj && obj->ObjectCls().m_data == g_CLR_RT_WellKnownTypes.m_AppDomain.m_data)
        {
            // managed appDomain is found.  Use it.
            res.SetObjectReference(ref->m_objectPtr);

            NANOCLR_SET_AND_LEAVE(S_OK);
        }
    }
    NANOCLR_FOREACH_NODE_END();

    {
        // Create the managed AppDomain in the destination AppDomain
        CLR_RT_HeapBlock *pRes;
        CLR_RT_ProtectFromGC gc(res);

        NANOCLR_CHECK_HRESULT(g_CLR_RT_ExecutionEngine.NewObjectFromIndex(res, g_CLR_RT_WellKnownTypes.m_AppDomain));

        pRes = res.Dereference();

        NANOCLR_CHECK_HRESULT(CLR_RT_ObjectToEvent_Source::CreateInstance(
            this,
            *pRes,
            pRes[Library_corlib_native_System_AppDomain::FIELD___appDomain]));

        pRes[Library_corlib_native_System_AppDomain::FIELD___friendlyName].SetObjectReference(m_strName);
    }

    NANOCLR_CLEANUP();

    g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomainSav);

    NANOCLR_CLEANUP_END();
}

CLR_RT_AppDomainAssembly *CLR_RT_AppDomain::FindAppDomainAssembly(CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    if (m_appDomainAssemblyLastAccess != NULL && m_appDomainAssemblyLastAccess->m_assembly == assm)
    {
        return m_appDomainAssemblyLastAccess;
    }

    NANOCLR_FOREACH_NODE(CLR_RT_AppDomainAssembly, appDomainAssembly, m_appDomainAssemblies)
    {
        if (appDomainAssembly->m_assembly == assm)
        {
            m_appDomainAssemblyLastAccess = appDomainAssembly;

            return appDomainAssembly;
        }
    }
    NANOCLR_FOREACH_NODE_END();

    return NULL;
}

void CLR_RT_AppDomain::Relocate()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_globalLock);
    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_strName);
    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_outOfMemoryException);
}

HRESULT CLR_RT_AppDomain::VerifyTypeIsLoaded(const CLR_RT_TypeDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_TypeDef_Instance inst;

    if (!inst.InitializeFromIndex(index))
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
    if (!FindAppDomainAssembly(inst.m_assm))
        NANOCLR_SET_AND_LEAVE(CLR_E_APPDOMAIN_MARSHAL_EXCEPTION);

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_AppDomain::MarshalObject(CLR_RT_HeapBlock &src, CLR_RT_HeapBlock &dst, CLR_RT_AppDomain *appDomainSrc)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    // This function marshals an object from appDomainSrc to 'this' AppDomain
    // If appDomainSrc == NULL, this uses the current AppDomain

    CLR_RT_AppDomain *appDomainDst = this;
    CLR_RT_HeapBlock *proxySrc = NULL;
    CLR_RT_HeapBlock *mbroSrc = NULL;
    bool fSimpleAssign = false;
    CLR_RT_TypeDef_Index indexVerify = g_CLR_RT_WellKnownTypes.m_Object;
    nanoClrDataType dtSrc = src.DataType();
    CLR_RT_AppDomain *appDomainSav = g_CLR_RT_ExecutionEngine.GetCurrentAppDomain();

    if (!appDomainSrc)
    {
        appDomainSrc = appDomainSav;
    }

    //
    // DATATYPE_LAST_PRIMITIVE_TO_MARSHAL note
    // We should think about allowing STRINGS to be shared across AD boundaries
    // Strings are read-only and it is safe to do this with some small restrictions
    // First, as with the Assembly unloading, some strings can actually point within an
    // assembly.  If we allow assemblies to be unloaded (and why shouldn't we, if they are
    // not in any AppDomain, than we need to deal with this case).  If we just
    // get copy the string on constructor, then we do not need to marshal strings
    // across AD boundaries.
    //
    fSimpleAssign = (appDomainSrc == appDomainDst);
    fSimpleAssign = fSimpleAssign || (dtSrc <= DATATYPE_LAST_PRIMITIVE_TO_MARSHAL);
    fSimpleAssign = fSimpleAssign || (dtSrc == DATATYPE_OBJECT && src.Dereference() == NULL);

#if !defined(NANOCLR_NO_ASSEMBLY_STRINGS)
    fSimpleAssign = fSimpleAssign || (dtSrc == DATATYPE_STRING && !src.StringAssembly());
#endif

    if (!fSimpleAssign)
    {
        if (dtSrc == DATATYPE_OBJECT)
        {
            CLR_RT_HeapBlock *ptr = src.Dereference();

            switch (ptr->DataType())
            {
                case DATATYPE_TRANSPARENT_PROXY:
                {
                    proxySrc = ptr;

                    NANOCLR_CHECK_HRESULT(proxySrc->TransparentProxyValidate());

                    indexVerify = proxySrc->TransparentProxyDereference()->ObjectCls();

                    if (proxySrc->TransparentProxyAppDomain() != appDomainDst)
                    {
                        // marshalling a transparent proxy object to a third AppDomain
                        // This makes the marshaling a simple assign of the DATATYPE_TRANSPARENT_PROXY heapblock
                        fSimpleAssign = true;
                    }
                }
                break;
                case DATATYPE_CLASS:
                {
                    CLR_RT_TypeDef_Instance inst;

                    if (inst.InitializeFromIndex(ptr->ObjectCls()))
                    {
                        if ((inst.CrossReference().m_flags &
                             CLR_RT_TypeDef_CrossReference::TD_CR_IsMarshalByRefObject) != 0)
                        {
                            indexVerify = inst;

                            mbroSrc = ptr;
                        }
                    }
                }
                break;
            }
        }
    }

    NANOCLR_CHECK_HRESULT(appDomainDst->VerifyTypeIsLoaded(indexVerify));

    if (fSimpleAssign)
    {
        dst.Assign(src);
    }
    else if (proxySrc != NULL)
    {
        // src is OBJECT->TRANSPARENT_PROXY->CLASS, and we are marshalling into 'this' appDomain
        // dst is OBJECT->CLASS
        _ASSERTE(proxySrc->TransparentProxyAppDomain() == appDomainDst);

        dst.SetObjectReference(proxySrc->TransparentProxyDereference());
    }
    else if (mbroSrc != NULL)
    {
        // src is a MarshalByRefObject that we are marshalling outside of its AppDomain
        // src is OBJECT->CLASS
        // dst is OBJECT->TRANSPARENT_PROXY->CLASS

        (void)g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomainDst);
        CLR_RT_HeapBlock *proxyDst =
            g_CLR_RT_ExecutionEngine.ExtractHeapBlocksForObjects(DATATYPE_TRANSPARENT_PROXY, 0, 1);
        CHECK_ALLOCATION(proxyDst);

        proxyDst->SetTransparentProxyReference(appDomainSrc, mbroSrc);
        dst.SetObjectReference(proxyDst);
    }
    else
    {
        CLR_RT_HeapBlock blk;
        blk.SetObjectReference(NULL);
        CLR_RT_ProtectFromGC gc(blk);
        bool fNoCompaction = CLR_EE_DBG_IS(NoCompaction);

        // Need to prevent compaction between serialization/deserialization.
        // Doesn't seem that currently compaction can actually occur during this time,
        // but just to be safe, we should prevent it.

        CLR_EE_DBG_SET(NoCompaction);
        NANOCLR_CHECK_HRESULT(
            CLR_RT_BinaryFormatter::Serialize(blk, src, NULL, CLR_RT_BinaryFormatter::c_Flags_Marshal));

        (void)g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomainDst);
        hr = CLR_RT_BinaryFormatter::Deserialize(dst, blk, NULL, NULL, CLR_RT_BinaryFormatter::c_Flags_Marshal);

        CLR_EE_DBG_RESTORE(NoCompaction, fNoCompaction);
    }

    NANOCLR_CLEANUP();

    (void)g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomainSav);

    NANOCLR_CLEANUP_END();
}

HRESULT CLR_RT_AppDomain::MarshalParameters(
    CLR_RT_HeapBlock *callerArgs,
    CLR_RT_HeapBlock *calleeArgs,
    int count,
    bool fOnReturn,
    CLR_RT_AppDomain *appDomainSrc)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_HeapBlock *src = fOnReturn ? calleeArgs : callerArgs;
    CLR_RT_HeapBlock *dst = fOnReturn ? callerArgs : calleeArgs;

    while (count-- > 0)
    {
        nanoClrDataType dtSrc = src->DataType();
        nanoClrDataType dtDst = dst->DataType();

        if (dtSrc == DATATYPE_BYREF || dtSrc == DATATYPE_ARRAY_BYREF)
        {
            CLR_RT_HeapBlock srcObj;
            CLR_RT_HeapBlock dstObj;

            if (fOnReturn)
            {
                srcObj.Assign(*src->Dereference());

                NANOCLR_CHECK_HRESULT(MarshalObject(srcObj, dstObj, appDomainSrc));

                // Move the marshaled object back into dst heapblock
                if (dtDst == DATATYPE_BYREF)
                {
                    dst->Dereference()->Assign(dstObj);
                }
                else
                {
                    dstObj.StoreToReference(*dst, 0);
                }
            }
            else //! fOnReturn
            {
                CLR_RT_HeapBlock *dstPtr = NULL;

                if (dtSrc == DATATYPE_BYREF)
                {
                    srcObj.Assign(*src->Dereference());
                }
                else
                {
                    srcObj.LoadFromReference(*src);
                }

                NANOCLR_CHECK_HRESULT(MarshalObject(srcObj, dstObj, appDomainSrc));

                // Need to copy DstObj onto the heap.
                dstPtr = g_CLR_RT_ExecutionEngine.ExtractHeapBlocksForObjects(DATATYPE_OBJECT, 0, 1);
                FAULT_ON_NULL(dstPtr);

                _ASSERTE(c_CLR_RT_DataTypeLookup[dstObj.DataType()].m_sizeInBytes != CLR_RT_DataTypeLookup::c_NA);

                dstPtr->Assign(dstObj);

                // Turn the OBJECT back into a BYREF
                dst->SetReference(*dstPtr);
            }
        }
        else // Not BYREF
        {
            if (!fOnReturn)
            {
                NANOCLR_CHECK_HRESULT(MarshalObject(*src, *dst, appDomainSrc));
            }
        }

        src++;
        dst++;
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_AppDomain::GetAssemblies(CLR_RT_HeapBlock &ref)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    int count = 0;
    CLR_RT_HeapBlock *pArray = NULL;

    for (int pass = 0; pass < 2; pass++)
    {
        NANOCLR_FOREACH_ASSEMBLY_IN_APPDOMAIN(this)
        {
            if (pass == 0)
            {
                count++;
            }
            else
            {
                CLR_RT_HeapBlock *hbObj;
                CLR_RT_Assembly_Index index;
                index.Set(pASSM->m_index);

                NANOCLR_CHECK_HRESULT(
                    g_CLR_RT_ExecutionEngine.NewObjectFromIndex(*pArray, g_CLR_RT_WellKnownTypes.m_Assembly));
                hbObj = pArray->Dereference();

                hbObj->SetReflection(index);

                pArray++;
            }
        }
        NANOCLR_FOREACH_ASSEMBLY_IN_APPDOMAIN_END();

        if (pass == 0)
        {
            NANOCLR_CHECK_HRESULT(
                CLR_RT_HeapBlock_Array::CreateInstance(ref, count, g_CLR_RT_WellKnownTypes.m_Assembly));

            pArray = (CLR_RT_HeapBlock *)ref.DereferenceArray()->GetFirstElement();
        }
    }

    NANOCLR_NOCLEANUP();
}

bool CLR_RT_AppDomain::IsLoaded()
{
    NATIVE_PROFILE_CLR_CORE();
    return m_state == CLR_RT_AppDomain::AppDomainState_Loaded;
}

//--//

HRESULT CLR_RT_AppDomainAssembly::CreateInstance(
    CLR_RT_AppDomain *appDomain,
    CLR_RT_Assembly *assm,
    CLR_RT_AppDomainAssembly *&appDomainAssembly)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    int size = CONVERTFROMSIZETOHEAPBLOCKS(sizeof(CLR_RT_AppDomainAssembly)) + assm->m_iStaticFields;

    appDomainAssembly = EVENTCACHE_EXTRACT_NODE_AS_BLOCKS(
        g_CLR_RT_EventCache,
        CLR_RT_AppDomainAssembly,
        DATATYPE_APPDOMAIN_ASSEMBLY,
        CLR_RT_HeapBlock::HB_InitializeToZero,
        size);
    CHECK_ALLOCATION(appDomainAssembly);

    NANOCLR_CHECK_HRESULT(appDomainAssembly->AppDomainAssembly_Initialize(appDomain, assm));

    NANOCLR_CLEANUP();

    if (FAILED(hr))
    {
        if (appDomainAssembly)
        {
            appDomainAssembly->DestroyInstance();
        }
    }

    NANOCLR_CLEANUP_END();
}

HRESULT CLR_RT_AppDomainAssembly::AppDomainAssembly_Initialize(CLR_RT_AppDomain *appDomain, CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_AppDomain *appDomainSav = g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomain);

    m_appDomain = appDomain;
    m_assembly = assm;
    m_flags = 0;
    m_pStaticFields = (CLR_RT_HeapBlock *)&this[1];

    /*
        The AppDomainAssembly gets linked before it is actually initialized, for two reasons.  First, it needs to be
        attached to the AppDomain in case a GC runs during the allocation of fields.  Second, this assembly needs to be
        loaded in the AppDomain when the static constructors are run.
    */

    appDomain->m_appDomainAssemblies.LinkAtBack(this);

    NANOCLR_CHECK_HRESULT(assm->Resolve_AllocateStaticFields(m_pStaticFields));

    if (!CLR_EE_DBG_IS_MASK(StateInitialize, StateMask))
    {
        // Only in the non-boot case should we do this. Otherwise, debug events can occur out of order (thread creation
        // of the static constructor before thread creation of the main thread.
        g_CLR_RT_ExecutionEngine.SpawnStaticConstructor(g_CLR_RT_ExecutionEngine.m_cctorThread);
    }

    NANOCLR_CLEANUP();

    if (FAILED(hr))
    {
        Unlink();
    }

    g_CLR_RT_ExecutionEngine.SetCurrentAppDomain(appDomainSav);

    NANOCLR_CLEANUP_END();
}

void CLR_RT_AppDomainAssembly::DestroyInstance()
{
    NATIVE_PROFILE_CLR_CORE();
    Unlink();

    g_CLR_RT_EventCache.Append_Node(this);
}

void CLR_RT_AppDomainAssembly::Relocate()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_GarbageCollector::Heap_Relocate(m_pStaticFields, m_assembly->m_iStaticFields);
}

#endif // NANOCLR_APPDOMAINS

//--//

struct TypeIndexLookup
{
    const char *nameSpace;
    const char *name;
    CLR_RT_TypeDef_Index *ptr;
};

static const TypeIndexLookup c_TypeIndexLookup[] = {
#define TIL(ns, nm, fld)                                                                                               \
    {                                                                                                                  \
        ns, nm, &g_CLR_RT_WellKnownTypes.fld                                                                           \
    }
    TIL("System", "Boolean", m_Boolean),
    TIL("System", "Char", m_Char),
    TIL("System", "SByte", m_Int8),
    TIL("System", "Byte", m_UInt8),
    TIL("System", "Int16", m_Int16),
    TIL("System", "UInt16", m_UInt16),
    TIL("System", "Int32", m_Int32),
    TIL("System", "UInt32", m_UInt32),
    TIL("System", "Int64", m_Int64),
    TIL("System", "UInt64", m_UInt64),
    TIL("System", "Single", m_Single),
    TIL("System", "Double", m_Double),
    TIL("System", "DateTime", m_DateTime),
    TIL("System", "TimeSpan", m_TimeSpan),
    TIL("System", "String", m_String),

    TIL("System", "Void", m_Void),
    TIL("System", "Object", m_Object),
    TIL("System", "ValueType", m_ValueType),
    TIL("System", "Enum", m_Enum),

    TIL("System", "AppDomainUnloadedException", m_AppDomainUnloadedException),
    TIL("System", "ArgumentNullException", m_ArgumentNullException),
    TIL("System", "ArgumentException", m_ArgumentException),
    TIL("System", "ArgumentOutOfRangeException", m_ArgumentOutOfRangeException),
    TIL("System", "Exception", m_Exception),
    TIL("System", "IndexOutOfRangeException", m_IndexOutOfRangeException),
    TIL("System", "InvalidCastException", m_InvalidCastException),
    TIL("System", "InvalidOperationException", m_InvalidOperationException),
    TIL("System", "NotSupportedException", m_NotSupportedException),
    TIL("System", "NotImplementedException", m_NotImplementedException),
    TIL("System", "NullReferenceException", m_NullReferenceException),
    TIL("System", "OutOfMemoryException", m_OutOfMemoryException),
    TIL("System", "ObjectDisposedException", m_ObjectDisposedException),
    TIL("System.Threading", "ThreadAbortException", m_ThreadAbortException),
    TIL("nanoFramework.Runtime.Native", "ConstraintException", m_ConstraintException),

    TIL("System", "Delegate", m_Delegate),
    TIL("System", "MulticastDelegate", m_MulticastDelegate),

    TIL("System", "Array", m_Array),
    TIL("System.Collections", "ArrayList", m_ArrayList),
    TIL("System", "ICloneable", m_ICloneable),
    TIL("System.Collections", "IList", m_IList),

    TIL("System.Reflection", "Assembly", m_Assembly),
    TIL("System", "Type", m_TypeStatic),
    TIL("System", "RuntimeType", m_Type),
    TIL("System.Reflection", "RuntimeConstructorInfo", m_ConstructorInfo),
    TIL("System.Reflection", "RuntimeMethodInfo", m_MethodInfo),
    TIL("System.Reflection", "RuntimeFieldInfo", m_FieldInfo),

    TIL("System", "WeakReference", m_WeakReference),

    TIL("nanoFramework.UI", "Bitmap", m_Bitmap),
    TIL("nanoFramework.UI", "Font", m_Font),
    TIL("nanoFramework.Touch", "TouchEvent", m_TouchEvent),
    TIL("nanoFramework.Touch", "TouchInput", m_TouchInput),

    TIL("System.Net.NetworkInformation", "NetworkInterface", m_NetworkInterface),
    TIL("System.Net.NetworkInformation", "Wireless80211Configuration", m_Wireless80211Configuration),
    TIL("System.Net.NetworkInformation", "WirelessAPConfiguration", m_WirelessAPConfiguration),
    TIL("System.Net.NetworkInformation", "WirelessAPStation", m_WirelessAPStation),

#if defined(NANOCLR_APPDOMAINS)
    TIL("System", "AppDomain", m_AppDomain),
    TIL("System", "MarshalByRefObject", m_MarshalByRefObject),
#endif

    TIL("System.Threading", "Thread", m_Thread),
    TIL("System.Resources", "ResourceManager", m_ResourceManager),

    TIL("System.Net.Sockets", "SocketException", m_SocketException),

    TIL("System.Device.I2c", "I2cTransferResult", m_I2cTransferResult),
    TIL("Windows.Devices.I2c", "I2cTransferResult", m_I2cTransferResult_old),

    TIL("nanoFramework.Hardware.Esp32.Rmt", "RmtCommand", m_RmtCommand),

#undef TIL
};

//--//

struct MethodIndexLookup
{
    const char *name;
    CLR_RT_TypeDef_Index *type;
    CLR_RT_MethodDef_Index *method;
};

static const MethodIndexLookup c_MethodIndexLookup[] = {
#define MIL(nm, type, method)                                                                                          \
    {                                                                                                                  \
        nm, &g_CLR_RT_WellKnownTypes.type, &g_CLR_RT_WellKnownMethods.method                                           \
    }

    MIL("GetObjectFromId", m_ResourceManager, m_ResourceManager_GetObjectFromId),
    MIL("GetObjectChunkFromId", m_ResourceManager, m_ResourceManager_GetObjectChunkFromId),

#undef MIL
};

void CLR_RT_Assembly::Resolve_TypeDef()
{
    NATIVE_PROFILE_CLR_CORE();
    const TypeIndexLookup *tilOuterClass = NULL;
    const TypeIndexLookup *til = c_TypeIndexLookup;

    for (size_t i = 0; i < ARRAYSIZE(c_TypeIndexLookup); i++, til++)
    {
        CLR_RT_TypeDef_Index &dst = *til->ptr;

        if (NANOCLR_INDEX_IS_INVALID(dst))
        {
            if (til->nameSpace == NULL)
            {
                if (tilOuterClass)
                {
                    FindTypeDef(til->name, tilOuterClass->ptr->Type(), dst);
                }
            }
            else
            {
                FindTypeDef(til->name, til->nameSpace, dst);
            }
        }

        if (til->nameSpace != NULL)
        {
            tilOuterClass = til;
        }
    }
}

void CLR_RT_Assembly::Resolve_MethodDef()
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_METHODDEF *md = GetMethodDef(0);

    for (int indexMethod = 0; indexMethod < m_pTablesSize[TBL_MethodDef]; indexMethod++, md++)
    {
        const MethodIndexLookup *mil = c_MethodIndexLookup;

        CLR_RT_MethodDef_Index index;
        index.Set(m_index, indexMethod);

        // Check for wellKnownMethods
        for (size_t ii = 0; ii < ARRAYSIZE(c_MethodIndexLookup); ii++, mil++)
        {
            CLR_RT_TypeDef_Index &indexType = *mil->type;
            CLR_RT_MethodDef_Index &mIndex = *mil->method;

            if (NANOCLR_INDEX_IS_VALID(indexType) && NANOCLR_INDEX_IS_INVALID(mIndex))
            {
                CLR_RT_TypeDef_Instance instType;

                _SIDE_ASSERTE(instType.InitializeFromIndex(indexType));

                if (instType.m_assm == this)
                {
                    if (!strcmp(GetString(md->Name), mil->name))
                    {
                        mIndex.m_data = index.m_data;
                    }
                }
            }
        }

        if (md->Flags & CLR_RECORD_METHODDEF::MD_EntryPoint)
        {
            g_CLR_RT_TypeSystem.m_entryPoint = index;
        }

        // link generic parameters
        if (md->GenericParamCount)
        {
            CLR_RT_GenericParam_CrossReference* gp = &m_pCrossReference_GenericParam[md->FirstGenericParam];

            // get generic parameter count for stop condition
            int num = md->GenericParamCount;
            CLR_UINT16 indexGenericParam = md->FirstGenericParam;

            for (; num; num--, gp++, indexGenericParam++)
            {
                CLR_RT_GenericParam_Index gpIndex;
                gpIndex.Set(m_index, indexGenericParam);

                gp->m_target = gpIndex;

                gp->m_data = indexMethod;
                gp->m_TypeOrMethodDef = TBL_MethodDef;

                CLR_RT_SignatureParser sub;
                if (sub.Initialize_GenericParamTypeSignature(this, GetGenericParam(indexGenericParam)))
                {
                    CLR_RT_SignatureParser::Element res;

                    // get generic param type
                    sub.Advance(res);

                    gp->DataType = res.DataType;
                    gp->Class = res.Class;
                }
                else
                {
                    gp->DataType = DATATYPE_VOID;

                    CLR_RT_TypeDef_Index td;
                    td.Clear();

                    gp->Class = td;
                }
            }
        }
    }
}

HRESULT CLR_RT_Assembly::Resolve_AllocateStaticFields(CLR_RT_HeapBlock *pStaticFields)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    const CLR_RECORD_FIELDDEF *fd = GetFieldDef(0);

    for (int i = 0; i < m_pTablesSize[TBL_FieldDef]; i++, fd++)
    {
        if (fd->Flags & CLR_RECORD_FIELDDEF::FD_Static)
        {
            CLR_RT_FieldDef_CrossReference &res = m_pCrossReference_FieldDef[i];

            NANOCLR_CHECK_HRESULT(g_CLR_RT_ExecutionEngine.InitializeReference(pStaticFields[res.m_offset], fd, this));
        }
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_Assembly::PrepareForExecution()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if ((m_flags & CLR_RT_Assembly::PreparingForExecution) != 0)
    {
        // Circular dependency
        _ASSERTE(false);

        NANOCLR_MSG_SET_AND_LEAVE(CLR_E_FAIL, L"Failed to prepare type system for execution\n");
    }

    if ((m_flags & CLR_RT_Assembly::PreparedForExecution) == 0)
    {
        int i;

        m_flags |= CLR_RT_Assembly::PreparingForExecution;

        ITERATE_THROUGH_RECORDS(this, i, AssemblyRef, ASSEMBLYREF)
        {
            _ASSERTE(dst->m_target != NULL);

            if (dst->m_target != NULL)
            {
                NANOCLR_CHECK_HRESULT(dst->m_target->PrepareForExecution());
            }
        }

#if defined(NANOCLR_APPDOMAINS)
        // Temporary solution.  All Assemblies get added to the current AppDomain
        // Which assemblies get loaded at boot, and when assemblies get added to AppDomain at runtime is
        // not yet determined/implemented

        NANOCLR_CHECK_HRESULT(g_CLR_RT_ExecutionEngine.GetCurrentAppDomain()->LoadAssembly(this));
#endif
    }

    NANOCLR_CLEANUP();

    // Only try once.  If this fails, then what?
    m_flags |= CLR_RT_Assembly::PreparedForExecution;
    m_flags &= ~CLR_RT_Assembly::PreparingForExecution;

    NANOCLR_CLEANUP_END();
}

//--//

CLR_UINT32 CLR_RT_Assembly::ComputeAssemblyHash()
{
    NATIVE_PROFILE_CLR_CORE();
    return m_header->ComputeAssemblyHash(m_szName, m_header->version);
}

CLR_UINT32 CLR_RT_Assembly::ComputeAssemblyHash(const CLR_RECORD_ASSEMBLYREF *ar)
{
    NATIVE_PROFILE_CLR_CORE();
    return m_header->ComputeAssemblyHash(GetString(ar->name), ar->version);
}

//--//

bool CLR_RT_Assembly::FindTypeDef(const char *name, const char *nameSpace, CLR_RT_TypeDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_TYPEDEF *target = GetTypeDef(0);
    int tblSize = m_pTablesSize[TBL_TypeDef];

    for (int i = 0; i < tblSize; i++, target++)
    {
        if (!target->HasValidEnclosingType())
        {
            const char *szNameSpace = GetString(target->NameSpace);
            const char *szName = GetString(target->Name);

            if (!strcmp(szName, name) && !strcmp(szNameSpace, nameSpace))
            {
                index.Set(m_index, i);

                return true;
            }
        }
    }

    index.Clear();

    return false;
}

bool CLR_RT_Assembly::FindTypeDef(const char *name, CLR_INDEX scope, CLR_RT_TypeDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_TYPEDEF *target = GetTypeDef(0);
    int tblSize = m_pTablesSize[TBL_TypeDef];

    for (int i = 0; i < tblSize; i++, target++)
    {
        if (target->EnclosingType() == scope)
        {
            const char *szName = GetString(target->Name);

            if (!strcmp(szName, name))
            {
                index.Set(m_index, i);

                return true;
            }
        }
    }

    index.Clear();

    return false;
}

bool CLR_RT_Assembly::FindTypeDef(CLR_UINT32 hash, CLR_RT_TypeDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_TypeDef_CrossReference *p = m_pCrossReference_TypeDef;
    CLR_UINT32 tblSize = m_pTablesSize[TBL_TypeDef];
    CLR_UINT32 i;

    for (i = 0; i < tblSize; i++, p++)
    {
        if (p->m_hash == hash)
            break;
    }

    if (i != tblSize)
    {
        index.Set(m_index, i);

        return true;
    }
    else
    {
        index.Clear();

        return false;
    }
}

bool CLR_RT_Assembly::FindGenericParam(CLR_INDEX typeSpecIndex, CLR_RT_GenericParam_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_GENERICPARAM* gp = this->GetGenericParam(0);
    int tblSize = this->m_pTablesSize[TBL_GenericParam];

    for (int i = 0; i < tblSize; i++, gp++)
    {
        CLR_RT_SignatureParser parserLeft;
        parserLeft.Initialize_GenericParamTypeSignature(this, gp);

        CLR_RT_SignatureParser parserRight;
        parserRight.Initialize_TypeSpec(this, this->GetTypeSpec(typeSpecIndex));

        if (CLR_RT_TypeSystem::MatchSignature(parserLeft, parserRight))
        {
            


            return true;
        }
    }

    index.Clear();

    return false;
}

bool CLR_RT_Assembly::FindGenericParamAtTypeDef(CLR_RT_MethodDef_Instance md, CLR_UINT32 genericParameterPosition, CLR_RT_GenericParam_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();

    CLR_INDEX indexType = md.CrossReference().GetOwner();

    CLR_INDEX paramIndex = GetTypeDef(indexType)->FirstGenericParam;

    // sanity check for valid parameter index
    if (paramIndex != CLR_EmptyIndex)
    {
        paramIndex += genericParameterPosition;

        index.Set(m_index, paramIndex);

        return true;
    }
    else
    {
        index.Clear();

        return false;
    }
}

bool CLR_RT_Assembly::FindGenericParamAtMethodDef(CLR_RT_MethodDef_Instance md, CLR_UINT32 genericParameterPosition, CLR_RT_GenericParam_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();

    CLR_INDEX paramIndex = GetMethodDef(md.Method())->FirstGenericParam;

    // sanity check for valid parameter index
    if (paramIndex != CLR_EmptyIndex)
    {
        paramIndex += genericParameterPosition;

        index.Set(m_index, paramIndex);

        return true;
    }
    else
    {
        index.Clear();

        return false;
    }
}

bool CLR_RT_Assembly::FindMethodSpecFromTypeSpec(CLR_INDEX typeSpecIndex, CLR_RT_MethodSpec_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_METHODSPEC* ms = this->GetMethodSpec(0);
    int tblSize = this->m_pTablesSize[TBL_MethodSpec];

    for (int i = 0; i < tblSize; i++, ms++)
    {
        if(ms->Container == typeSpecIndex)
        {
            index.Set(m_index, i);

            return true;
        }
    }

    index.Clear();

    return false;
}

//--//

static bool local_FindFieldDef(
    CLR_RT_Assembly *assm,
    CLR_UINT32 first,
    CLR_UINT32 num,
    const char *szText,
    CLR_RT_Assembly *base,
    CLR_INDEX sig,
    CLR_RT_FieldDef_Index &res)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_FIELDDEF *fd = assm->GetFieldDef(first);

    for (CLR_UINT32 i = 0; i < num; i++, fd++)
    {
        const char *fieldName = assm->GetString(fd->Name);

        if (!strcmp(fieldName, szText))
        {
            if (base)
            {
                CLR_RT_SignatureParser parserLeft;
                parserLeft.Initialize_FieldDef(assm, fd);
                CLR_RT_SignatureParser parserRight;
                parserRight.Initialize_FieldDef(base, base->GetSignature(sig));

                if (CLR_RT_TypeSystem::MatchSignature(parserLeft, parserRight) == false)
                {
                    continue;
                }
            }

            res.Set(assm->m_index, first + i);

            return true;
        }
    }

    res.Clear();

    return false;
}

bool CLR_RT_Assembly::FindFieldDef(
    const CLR_RECORD_TYPEDEF *td,
    const char *name,
    CLR_RT_Assembly *base,
    CLR_INDEX sig,
    CLR_RT_FieldDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    if (local_FindFieldDef(this, td->FirstInstanceField, td->InstanceFieldsCount, name, base, sig, index))
        return true;
    if (local_FindFieldDef(this, td->FirstStaticField, td->StaticFieldsCount, name, base, sig, index))
        return true;

    index.Clear();

    return false;
}

bool CLR_RT_Assembly::FindFieldDef(
    const CLR_RECORD_TYPESPEC* ts,
    const char* name,
    CLR_RT_Assembly* base,
    CLR_SIG sig,
    CLR_RT_FieldDef_Index& index)
{
    (void)ts;

    NATIVE_PROFILE_CLR_CORE();

    const CLR_RECORD_FIELDDEF* fd = GetFieldDef(0);

    for (int i = 0; i < m_pTablesSize[TBL_FieldDef]; i++, fd++)
    {
        const char* fieldName = GetString(fd->Name);

        if (!strcmp(fieldName, name))
        {
            bool fMatch = true;

            if (CLR_SIG_INVALID != sig)
            {
                CLR_RT_SignatureParser parserLeft;
                parserLeft.Initialize_FieldSignature(this, fd);
                CLR_RT_SignatureParser parserRight;
                parserRight.Initialize_FieldSignature(base, base->GetSignature(sig));

                fMatch = CLR_RT_TypeSystem::MatchSignature(parserLeft, parserRight);
            }

            if (fMatch)
            {
                index.Set(m_index, i);

                return true;
            }
        }
    }

    index.Clear();

    return false;
}

bool CLR_RT_Assembly::FindMethodDef(
    const CLR_RECORD_TYPEDEF *td,
    const char *name,
    CLR_RT_Assembly *base,
    CLR_SIG sig,
    CLR_RT_MethodDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    int i;
    int num = td->VirtualMethodCount + td->InstanceMethodCount + td->StaticMethodCount;
    const CLR_RECORD_METHODDEF *md = GetMethodDef(td->FirstMethod);

    for (i = 0; i < num; i++, md++)
    {
        const char *methodName = GetString(md->Name);

        if (!strcmp(methodName, name))
        {
            bool fMatch = true;

            if (CLR_SIG_INVALID != sig)
            {
                CLR_RT_SignatureParser parserLeft;
                parserLeft.Initialize_MethodSignature(this, md);
                CLR_RT_SignatureParser parserRight;
                parserRight.Initialize_MethodSignature(base, base->GetSignature(sig));

                fMatch = CLR_RT_TypeSystem::MatchSignature(parserLeft, parserRight);
            }

            if (fMatch)
            {
                index.Set(m_index, i + td->FirstMethod);

                return true;
            }
        }
    }

    index.Clear();

    return false;
}

bool CLR_RT_Assembly::FindMethodDef(
    const CLR_RECORD_TYPESPEC* ts,
    const char* name,
    CLR_RT_Assembly* base,
    CLR_SIG sig,
    CLR_RT_MethodDef_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();

    CLR_RT_TypeSpec_Index tsIndex;
    base->FindTypeSpec(base->GetSignature(ts->Sig), tsIndex);

    CLR_RT_TypeSpec_Instance tsInstance;
    tsInstance.InitializeFromIndex(tsIndex);

    const CLR_RECORD_TYPEDEF* td = (const CLR_RECORD_TYPEDEF*)base->GetTable(TBL_TypeDef);
    td += tsInstance.TypeDefIndex;

    return CLR_RT_Assembly::FindMethodDef(
        td,
        name,
        base,
        sig,
        index);
}

bool CLR_RT_Assembly::FindTypeSpec(CLR_PMETADATA sig, CLR_RT_TypeSpec_Index& index)
{
    NATIVE_PROFILE_CLR_CORE();

    for (int i = 0; i < m_pTablesSize[TBL_TypeSpec]; i++)
    {
        const CLR_RECORD_TYPESPEC* ts = GetTypeSpec(i);

        CLR_RT_SignatureParser parserLeft;
        parserLeft.Initialize_TypeSpec(this, sig);
        CLR_RT_SignatureParser parserRight;
        parserRight.Initialize_TypeSpec(this, ts);

        if (CLR_RT_TypeSystem::MatchSignature(parserLeft, parserRight))
        {
            // found it!
            // set TypeSpec index
            index.Set(this->m_index, i);

            // need to advance the signature to consume it
            while (parserLeft.Signature != sig)
            {
                sig++;
            }

            return true;
        }
    }

    index.Clear();
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CLR_RT_Assembly::FindMethodBoundaries(CLR_INDEX i, CLR_OFFSET &start, CLR_OFFSET &end)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_METHODDEF *p = GetMethodDef(i);

    if (p->RVA == CLR_EmptyIndex)
        return false;

    start = p->RVA;

    while (true)
    {
        p++;
        i++;

        if (i == m_pTablesSize[TBL_MethodDef])
        {
            end = m_pTablesSize[TBL_ByteCode];
            break;
        }

        if (p->RVA != CLR_EmptyIndex)
        {
            end = p->RVA;
            break;
        }
    }

    return true;
}

bool CLR_RT_Assembly::FindNextStaticConstructor(CLR_RT_MethodDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    _ASSERTE(m_index == index.Assembly());

    for (int i = index.Method(); i < m_pTablesSize[TBL_MethodDef]; i++)
    {
        const CLR_RECORD_METHODDEF *md = GetMethodDef(i);

        index.Set(m_index, i);

        if (md->Flags & CLR_RECORD_METHODDEF::MD_StaticConstructor)
        {
            return true;
        }
    }

    index.Clear();
    return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

HRESULT CLR_RT_Assembly::Resolve_ComputeHashes()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    const CLR_RECORD_TYPEDEF *src = GetTypeDef(0);
    CLR_RT_TypeDef_CrossReference *dst = m_pCrossReference_TypeDef;

    for (int i = 0; i < m_pTablesSize[TBL_TypeDef]; i++, src++, dst++)
    {
        CLR_RT_TypeDef_Index index;
        index.Set(m_index, i);
        CLR_RT_TypeDef_Instance inst;
        inst.InitializeFromIndex(index);
        CLR_UINT32 hash = ComputeHashForName(index, 0);

        while (NANOCLR_INDEX_IS_VALID(inst))
        {
            const CLR_RECORD_TYPEDEF *target = inst.m_target;
            const CLR_RECORD_FIELDDEF *fd = inst.m_assm->GetFieldDef(target->FirstInstanceField);

            for (int j = 0; j < target->InstanceFieldsCount; j++, fd++)
            {
                if ((fd->Flags & CLR_RECORD_FIELDDEF::FD_NotSerialized) == 0)
                {
                    CLR_RT_SignatureParser parser;
                    parser.Initialize_FieldDef(inst.m_assm, fd);
                    CLR_RT_SignatureParser::Element res;

                    NANOCLR_CHECK_HRESULT(parser.Advance(res));

                    while (res.Levels -- > 0)
                    {
                        hash = ComputeHashForType(DATATYPE_SZARRAY, hash);
                    }

                    hash = ComputeHashForType(res.DataType, hash);

                    if ((res.DataType == DATATYPE_VALUETYPE) || (res.DataType == DATATYPE_CLASS))
                    {
                        hash = ComputeHashForName(res.Class, hash);
                    }

                    const char *fieldName = inst.m_assm->GetString(fd->Name);

                    hash = SUPPORT_ComputeCRC(fieldName, (CLR_UINT32)hal_strlen_s(fieldName), hash);
                }
            }

            inst.SwitchToParent();
        }

        dst->m_hash = hash ? hash : 0xFFFFFFFF; // Don't allow zero as an hash value!!
    }

    NANOCLR_NOCLEANUP();
}

CLR_UINT32 CLR_RT_Assembly::ComputeHashForName(const CLR_RT_TypeDef_Index &td, CLR_UINT32 hash)
{
    NATIVE_PROFILE_CLR_CORE();
    char rgBuffer[512];
    char *szBuffer = rgBuffer;
    size_t iBuffer = MAXSTRLEN(rgBuffer);

    g_CLR_RT_TypeSystem.BuildTypeName(td, szBuffer, iBuffer);

    CLR_UINT32 hashPost = SUPPORT_ComputeCRC(rgBuffer, (int)(MAXSTRLEN(rgBuffer) - iBuffer), hash);

    return hashPost;
}

CLR_UINT32 CLR_RT_Assembly::ComputeHashForType(nanoClrDataType et, CLR_UINT32 hash)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_UINT8 val = (CLR_UINT8)CLR_RT_TypeSystem::MapDataTypeToElementType(et);

    CLR_UINT32 hashPost = SUPPORT_ComputeCRC(&val, sizeof(val), hash);

    return hashPost;
}

//--//

CLR_RT_HeapBlock *CLR_RT_Assembly::GetStaticField(const int index)
{
    NATIVE_PROFILE_CLR_CORE();

#if defined(NANOCLR_APPDOMAINS)

    CLR_RT_AppDomainAssembly *adAssm = g_CLR_RT_ExecutionEngine.GetCurrentAppDomain()->FindAppDomainAssembly(this);

    _ASSERTE(adAssm);

    return &adAssm->m_pStaticFields[index];

#else

    return &m_pStaticFields[index];

#endif
}

//--//

void CLR_RT_Assembly::Relocate()
{
    NATIVE_PROFILE_CLR_CORE();

#if !defined(NANOCLR_APPDOMAINS)
    CLR_RT_GarbageCollector::Heap_Relocate(m_pStaticFields, m_iStaticFields);
#endif

    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_header);
    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_szName);
    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_pFile);
    CLR_RT_GarbageCollector::Heap_Relocate((void **)&m_nativeCode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CLR_RT_TypeSystem::TypeSystem_Initialize()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_CLEAR(g_CLR_RT_TypeSystem);
    NANOCLR_CLEAR(g_CLR_RT_WellKnownTypes);
}

void CLR_RT_TypeSystem::TypeSystem_Cleanup()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_FOREACH_ASSEMBLY(*this)
    {
        pASSM->DestroyInstance();

        *ppASSM = NULL;
    }
    NANOCLR_FOREACH_ASSEMBLY_END();

    m_assembliesMax = 0;
}

//--//

void CLR_RT_TypeSystem::Link(CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_FOREACH_ASSEMBLY_NULL(*this)
    {
        *ppASSM = assm;

        assm->m_index = index;

        PostLinkageProcessing(assm);

        if (m_assembliesMax < index)
            m_assembliesMax = index;

        return;
    }
    NANOCLR_FOREACH_ASSEMBLY_NULL_END();
}

void CLR_RT_TypeSystem::PostLinkageProcessing(CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    if (!strcmp(assm->m_szName, "mscorlib"))
    {
        m_assemblyMscorlib = assm;
    }
    if (!strcmp(assm->m_szName, "nanoFramework.Runtime.Native"))
    {
        m_assemblyNative = assm;
    }
}

CLR_RT_Assembly *CLR_RT_TypeSystem::FindAssembly(const char *szName, const CLR_RECORD_VERSION *ver, bool fExact)
{
    NATIVE_PROFILE_CLR_CORE();

    NANOCLR_FOREACH_ASSEMBLY(*this)
    {
        if (!strcmp(pASSM->m_szName, szName))
        {
            // if there is no version information, anything goes
            if (NULL == ver)
            {
                return pASSM;
            }
            // exact match requested: must take into accoutn all numbers in the version
            else if (fExact)
            {
                if (0 == memcmp(&pASSM->m_header->version, ver, sizeof(*ver)))
                {
                    return pASSM;
                }
            }
            // exact match was NOT required but still there version information,
            // we will enforce only the first two number because (by convention)
            // only the minor field is required to be bumped when native assemblies change CRC
            else if (
                ver->iMajorVersion == pASSM->m_header->version.iMajorVersion &&
                ver->iMinorVersion == pASSM->m_header->version.iMinorVersion)
            {
                return pASSM;
            }
        }
    }
    NANOCLR_FOREACH_ASSEMBLY_END();

    return NULL;
}

bool CLR_RT_TypeSystem::FindTypeDef(
    const char *name,
    const char *nameSpace,
    CLR_RT_Assembly *assm,
    CLR_RT_TypeDef_Index &res)
{
    NATIVE_PROFILE_CLR_CORE();

    if (assm)
    {
        NANOCLR_FOREACH_ASSEMBLY_IN_CURRENT_APPDOMAIN(*this)
        {
            if (pASSM->IsSameAssembly(*assm) && pASSM->FindTypeDef(name, nameSpace, res))
                return true;
        }
        NANOCLR_FOREACH_ASSEMBLY_IN_CURRENT_APPDOMAIN_END();

        res.Clear();

        return false;
    }

    return FindTypeDef(name, nameSpace, res);
}

bool CLR_RT_TypeSystem::FindTypeDef(const char *name, const char *nameSpace, CLR_RT_TypeDef_Index &res)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_FOREACH_ASSEMBLY_IN_CURRENT_APPDOMAIN(*this)
    {
        if (pASSM->FindTypeDef(name, nameSpace, res))
            return true;
    }
    NANOCLR_FOREACH_ASSEMBLY_IN_CURRENT_APPDOMAIN_END();

    res.Clear();
    return false;
}

bool CLR_RT_TypeSystem::FindTypeDef(CLR_UINT32 hash, CLR_RT_TypeDef_Index &res)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_FOREACH_ASSEMBLY_IN_CURRENT_APPDOMAIN(*this)
    {
        if (pASSM->FindTypeDef(hash, res))
            return true;
    }
    NANOCLR_FOREACH_ASSEMBLY_IN_CURRENT_APPDOMAIN_END();

    res.Clear();
    return false;
}

bool CLR_RT_TypeSystem::FindTypeDef(const char *szClass, CLR_RT_Assembly *assm, CLR_RT_TypeDef_Index &res)
{
    (void)szClass;
    (void)assm;
    (void)res;

    NATIVE_PROFILE_CLR_CORE();

    char rgName[MAXTYPENAMELEN];
    char rgNamespace[MAXTYPENAMELEN];

    if (hal_strlen_s(szClass) < ARRAYSIZE(rgNamespace))
    {
        const char *szPtr = szClass;
        const char *szPtr_LastDot = NULL;
        const char *szPtr_FirstSubType = NULL;
        char c;
        size_t len;

        while (true)
        {
            c = szPtr[0];
            if (!c)
                break;

            if (c == '.')
            {
                szPtr_LastDot = szPtr;
            }
            else if (c == '+')
            {
                szPtr_FirstSubType = szPtr;
                break;
            }

            szPtr++;
        }

        if (szPtr_LastDot)
        {
            len = szPtr_LastDot++ - szClass;
            hal_strncpy_s(rgNamespace, ARRAYSIZE(rgNamespace), szClass, len);
            len = szPtr - szPtr_LastDot;
            hal_strncpy_s(rgName, ARRAYSIZE(rgName), szPtr_LastDot, len);
        }
        else
        {
            rgNamespace[0] = 0;
            hal_strcpy_s(rgName, ARRAYSIZE(rgName), szClass);
        }

        if (FindTypeDef(rgName, rgNamespace, assm, res))
        {
            //
            // Found the containing type, let's look for the nested type.
            //
            if (szPtr_FirstSubType)
            {
                CLR_RT_TypeDef_Instance inst;

                do
                {
                    szPtr = ++szPtr_FirstSubType;

                    while (true)
                    {
                        c = szPtr_FirstSubType[0];
                        if (!c)
                            break;

                        if (c == '+')
                            break;

                        szPtr_FirstSubType++;
                    }

                    len = szPtr_FirstSubType - szPtr;
                    hal_strncpy_s(rgName, ARRAYSIZE(rgName), szPtr, len);

                    inst.InitializeFromIndex(res);

                    if (inst.m_assm->FindTypeDef(rgName, res.Type(), res) == false)
                    {
                        return false;
                    }

                } while (c == '+');
            }

            return true;
        }
    }

    res.Clear();

    return false;
}

bool CLR_RT_TypeSystem::FindTypeDef(const char *szClass, CLR_RT_Assembly *assm, CLR_RT_ReflectionDef_Index &reflex)
{
    (void)szClass;
    (void)assm;
    (void)reflex;

    NATIVE_PROFILE_CLR_CORE();

    // UNDONE: FIXME
    // char rgName     [ MAXTYPENAMELEN ];
    // char rgNamespace[ MAXTYPENAMELEN ];
    // CLR_RT_TypeDef_Index res;

    // if(hal_strlen_s(szClass) < ARRAYSIZE(rgNamespace))
    // {
    //     const char* szPtr              = szClass;
    //     const char* szPtr_LastDot      = NULL;
    //     const char* szPtr_FirstSubType = NULL;
    //     char   c;
    //     size_t len;
    //     bool arrayType = false;

    //     while(true)
    //     {
    //         c = szPtr[ 0 ]; if(!c) break;

    //         if(c == '.')
    //         {
    //             szPtr_LastDot = szPtr;
    //         }
    //         else if(c == '+')
    //         {
    //             szPtr_FirstSubType = szPtr;
    //             break;
    //         }
    //         else if(c == '[')
    //         {
    //             char ch = szPtr[ 1 ];
    //             if (ch == ']')
    //             {
    //                 arrayType = true;
    //                 break;
    //             }
    //         }

    //         szPtr++;
    //     }

    //     if(szPtr_LastDot)
    //     {
    //         len = szPtr_LastDot++ - szClass      ; hal_strncpy_s( rgNamespace, ARRAYSIZE(rgNamespace), szClass      ,
    //         len ); len = szPtr           - szPtr_LastDot; hal_strncpy_s( rgName     , ARRAYSIZE(rgName     ),
    //         szPtr_LastDot, len );
    //     }
    //     else
    //     {
    //         rgNamespace[ 0 ] = 0;
    //         hal_strcpy_s( rgName, ARRAYSIZE(rgName), szClass );
    //     }

    //     if(FindTypeDef( rgName, rgNamespace, assm, res ))
    //     {
    //         //
    //         // Found the containing type, let's look for the nested type.
    //         //
    //         if(szPtr_FirstSubType)
    //         {
    //             CLR_RT_TypeDef_Instance inst;

    //             do
    //             {
    //                 szPtr = ++szPtr_FirstSubType;

    //                 while(true)
    //                 {
    //                     c = szPtr_FirstSubType[ 0 ]; if(!c) break;

    //                     if(c == '+') break;

    //                     szPtr_FirstSubType++;
    //                 }

    //                 len = szPtr_FirstSubType - szPtr; hal_strncpy_s( rgName, ARRAYSIZE(rgName), szPtr, len );

    //                 inst.InitializeFromIndex( res );

    //                 if(inst.m_assm->FindTypeDef( rgName, res.Type(), res ) == false)
    //                 {
    //                     return false;
    //                 }

    //             } while(c == '+');
    //         }

    //         reflex.m_kind        = REFLECTION_TYPE;
    //         // make sure this works for multidimensional arrays.
    //         reflex.m_levels      = arrayType ? 1 : 0;
    //         reflex.m_data.m_type = res;
    //         return true;
    //     }
    // }
    //
    // res.Clear();

    return false;
}

//--//

int
#if defined(_MSC_VER)
    __cdecl
#endif
    CompareResource(const void *p1, const void *p2)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_RESOURCE *resource1 = (const CLR_RECORD_RESOURCE *)p1;
    const CLR_RECORD_RESOURCE *resource2 = (const CLR_RECORD_RESOURCE *)p2;

    return (int)resource1->id - (int)resource2->id;
}

HRESULT CLR_RT_TypeSystem::LocateResourceFile(
    CLR_RT_Assembly_Instance assm,
    const char *name,
    CLR_INT32 &indexResourceFile)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_Assembly *pAssm = assm.m_assm;

    for (indexResourceFile = 0; indexResourceFile < pAssm->m_pTablesSize[TBL_ResourcesFiles]; indexResourceFile++)
    {
        const CLR_RECORD_RESOURCE_FILE *resourceFile = pAssm->GetResourceFile(indexResourceFile);

        if (!strcmp(pAssm->GetString(resourceFile->name), name))
        {
            NANOCLR_SET_AND_LEAVE(S_OK);
        }
    }

    indexResourceFile = -1;

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::LocateResource(
    CLR_RT_Assembly_Instance assm,
    CLR_INT32 indexResourceFile,
    CLR_INT16 id,
    const CLR_RECORD_RESOURCE *&res,
    CLR_UINT32 &size)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_Assembly *pAssm = assm.m_assm;
    const CLR_RECORD_RESOURCE_FILE *resourceFile;
    CLR_RECORD_RESOURCE resourceT;
    const CLR_RECORD_RESOURCE *resNext;
    const CLR_RECORD_RESOURCE *resZero;

    res = NULL;
    size = 0;

    if (indexResourceFile < 0 || indexResourceFile >= pAssm->m_pTablesSize[TBL_ResourcesFiles])
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);

    resourceFile = pAssm->GetResourceFile(indexResourceFile);

    _ASSERTE(resourceFile->numberOfResources > 0);

    resZero = pAssm->GetResource(resourceFile->offset);

    resourceT.id = id;

    res = (const CLR_RECORD_RESOURCE *)
        bsearch(&resourceT, resZero, resourceFile->numberOfResources, sizeof(CLR_RECORD_RESOURCE), CompareResource);

    if (res != NULL)
    {
        // compute size here...
        // assert not the last resource
        _ASSERTE(res + 1 <= pAssm->GetResource(pAssm->m_pTablesSize[TBL_Resources] - 1));
        resNext = res + 1;

        size = resNext->offset - res->offset;

        // deal with alignment.
        size -= (resNext->flags & CLR_RECORD_RESOURCE::FLAGS_PaddingMask);
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::ResolveAll()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    bool fOutput = false;

    while (true)
    {
        bool fGot = false;
        bool fNeedResolution = false;

        NANOCLR_FOREACH_ASSEMBLY(*this)
        {
            if ((pASSM->m_flags & CLR_RT_Assembly::Resolved) == 0)
            {
                fNeedResolution = true;

                if (pASSM->Resolve_AssemblyRef(fOutput))
                {
                    fGot = true;

                    pASSM->m_flags |= CLR_RT_Assembly::Resolved;

                    NANOCLR_CHECK_HRESULT(pASSM->Resolve_TypeRef());
                    NANOCLR_CHECK_HRESULT(pASSM->Resolve_FieldRef());
                    NANOCLR_CHECK_HRESULT(pASSM->Resolve_MethodRef());
                    /********************/ pASSM->Resolve_TypeDef();
                    /********************/ pASSM->Resolve_MethodDef();
                    NANOCLR_CHECK_HRESULT(pASSM->Resolve_TypeSpec());
                    /********************/ pASSM->Resolve_Link();
                    NANOCLR_CHECK_HRESULT(pASSM->Resolve_ComputeHashes());

#if !defined(NANOCLR_APPDOMAINS)
                    NANOCLR_CHECK_HRESULT(pASSM->Resolve_AllocateStaticFields(pASSM->m_pStaticFields));
#endif

                    pASSM->m_flags |= CLR_RT_Assembly::ResolutionCompleted;
                }
            }
        }
        NANOCLR_FOREACH_ASSEMBLY_END();

        if (fOutput == true)
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_TYPE_UNAVAILABLE);
        }

        if (fGot == false)
        {
            if (fNeedResolution)
            {
#if !defined(BUILD_RTM)
                CLR_Debug::Printf("Link failure: some assembly references cannot be resolved!!\r\n\r\n");
#endif

                fOutput = true;
            }
            else
            {
                break;
            }
        }
    }

#if !defined(BUILD_RTM)

    if (s_CLR_RT_fTrace_AssemblyOverhead >= c_CLR_RT_Trace_Info)
    {
        {
            int pTablesSize[TBL_Max];
            memset(pTablesSize, 0, sizeof(pTablesSize));
            CLR_RT_Assembly::Offsets offsets;
            memset(&offsets, 0, sizeof(offsets));

            size_t iStaticFields = 0;
            size_t iTotalRamSize = 0;
            size_t iTotalRomSize = 0;
            size_t iMetaData = 0;

            NANOCLR_FOREACH_ASSEMBLY(*this)
            {
                offsets.iBase += ROUNDTOMULTIPLE(sizeof(CLR_RT_Assembly), CLR_UINT32);
                offsets.iAssemblyRef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_AssemblyRef] * sizeof(CLR_RT_AssemblyRef_CrossReference),
                    CLR_UINT32);
                offsets.iTypeRef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_TypeRef] * sizeof(CLR_RT_TypeRef_CrossReference),
                    CLR_UINT32);
                offsets.iFieldRef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_FieldRef] * sizeof(CLR_RT_FieldRef_CrossReference),
                    CLR_UINT32);
                offsets.iMethodRef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_MethodRef] * sizeof(CLR_RT_MethodRef_CrossReference),
                    CLR_UINT32);
                offsets.iTypeDef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_TypeDef] * sizeof(CLR_RT_TypeDef_CrossReference),
                    CLR_UINT32);
                offsets.iFieldDef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_FieldDef] * sizeof(CLR_RT_FieldDef_CrossReference),
                    CLR_UINT32);
                offsets.iMethodDef += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_MethodDef] * sizeof(CLR_RT_MethodDef_CrossReference),
                    CLR_UINT32);
                offsets.iGenericParam += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_GenericParam] * sizeof(CLR_RT_GenericParam_CrossReference),
                    CLR_UINT32);

#if !defined(NANOCLR_APPDOMAINS)
                offsets.iStaticFields += ROUNDTOMULTIPLE(pASSM->m_iStaticFields * sizeof(CLR_RT_HeapBlock), CLR_UINT32);
#endif

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
                offsets.iDebuggingInfoMethods += ROUNDTOMULTIPLE(
                    pASSM->m_pTablesSize[TBL_MethodDef] * sizeof(CLR_RT_MethodDef_DebuggingInfo),
                    CLR_UINT32);
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)

                iMetaData += pASSM->m_header->SizeOfTable(TBL_AssemblyRef) + pASSM->m_header->SizeOfTable(TBL_TypeRef) +
                             pASSM->m_header->SizeOfTable(TBL_FieldRef) + pASSM->m_header->SizeOfTable(TBL_MethodRef) +
                             pASSM->m_header->SizeOfTable(TBL_TypeDef) + pASSM->m_header->SizeOfTable(TBL_FieldDef) +
                             pASSM->m_header->SizeOfTable(TBL_MethodDef) + 
                             pASSM->m_header->SizeOfTable(TBL_GenericParam) +pASSM->m_header->SizeOfTable(TBL_TypeSpec) +
                             pASSM->m_header->SizeOfTable(TBL_Attributes) + 
                             pASSM->m_header->SizeOfTable(TBL_Signatures);

                for (int tbl = 0; tbl < TBL_Max; tbl++)
                {
                    pTablesSize[tbl] += pASSM->m_pTablesSize[tbl];
                }

                iTotalRomSize += pASSM->m_header->TotalSize();

                iStaticFields += pASSM->m_iStaticFields;
            }
            NANOCLR_FOREACH_ASSEMBLY_END();

            iTotalRamSize = offsets.iBase + offsets.iAssemblyRef + offsets.iTypeRef + offsets.iFieldRef +
                            offsets.iMethodRef + offsets.iTypeDef + offsets.iFieldDef + offsets.iMethodDef +
                            offsets.iGenericParam;

#if !defined(NANOCLR_APPDOMAINS)
            iTotalRamSize += offsets.iStaticFields;
#endif

            CLR_Debug::Printf(
                "\r\nTotal: (%d RAM - %d ROM - %d METADATA)\r\n\r\n",
                iTotalRamSize,
                iTotalRomSize,
                iMetaData);

            CLR_Debug::Printf(
                "   AssemblyRef     = %6d bytes (%5d elements)\r\n",
                offsets.iAssemblyRef,
                pTablesSize[TBL_AssemblyRef]);
            CLR_Debug::Printf(
                "   TypeRef         = %6d bytes (%5d elements)\r\n",
                offsets.iTypeRef,
                pTablesSize[TBL_TypeRef]);
            CLR_Debug::Printf(
                "   FieldRef        = %6d bytes (%5d elements)\r\n",
                offsets.iFieldRef,
                pTablesSize[TBL_FieldRef]);
            CLR_Debug::Printf(
                "   MethodRef       = %6d bytes (%5d elements)\r\n",
                offsets.iMethodRef,
                pTablesSize[TBL_MethodRef]);
            CLR_Debug::Printf(
                "   TypeDef         = %6d bytes (%5d elements)\r\n",
                offsets.iTypeDef,
                pTablesSize[TBL_TypeDef]);
            CLR_Debug::Printf(
                "   FieldDef        = %6d bytes (%5d elements)\r\n",
                offsets.iFieldDef,
                pTablesSize[TBL_FieldDef]);
            CLR_Debug::Printf(
                "   MethodDef       = %6d bytes (%5d elements)\r\n",
                offsets.iMethodDef,
                pTablesSize[TBL_MethodDef]);
            CLR_Debug::Printf(
                "   GenericParam    = %6d bytes (%5d elements)\r\n",
                offsets.iGenericParam,
                pTablesSize[TBL_GenericParam]);
            CLR_Debug::Printf(
                "   MethodSpec      = %6d bytes (%5d elements)\r\n",
                offsets.iMethodSpec,
                pTablesSize[TBL_MethodSpec]);

#if !defined(NANOCLR_APPDOMAINS)
            CLR_Debug::Printf(
                "   StaticFields    = %6d bytes (%5d elements)\r\n",
                offsets.iStaticFields,
                iStaticFields);
#endif

            CLR_Debug::Printf("\r\n");

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
            CLR_Debug::Printf("   DebuggingInfo   = %6d bytes\r\n", offsets.iDebuggingInfoMethods);
            CLR_Debug::Printf("\r\n");
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)

            CLR_Debug::Printf(
                "   Attributes      = %6d bytes (%5d elements)\r\n",
                pTablesSize[TBL_Attributes] * sizeof(CLR_RECORD_ATTRIBUTE),
                pTablesSize[TBL_Attributes]);
            CLR_Debug::Printf(
                "   TypeSpec        = %6d bytes (%5d elements)\r\n",
                pTablesSize[TBL_TypeSpec] * sizeof(CLR_RECORD_TYPESPEC),
                pTablesSize[TBL_TypeSpec]);
            CLR_Debug::Printf(
                "   Resources Files = %6d bytes (%5d elements)\r\n",
                pTablesSize[TBL_ResourcesFiles] * sizeof(CLR_RECORD_RESOURCE_FILE),
                pTablesSize[TBL_ResourcesFiles]);
            CLR_Debug::Printf(
                "   Resources       = %6d bytes (%5d elements)\r\n",
                pTablesSize[TBL_Resources] * sizeof(CLR_RECORD_RESOURCE),
                pTablesSize[TBL_Resources]);
            CLR_Debug::Printf("   Resources Data  = %6d bytes\r\n", pTablesSize[TBL_ResourcesData]);
            CLR_Debug::Printf("   Strings         = %6d bytes\r\n", pTablesSize[TBL_Strings]);
            CLR_Debug::Printf("   Signatures      = %6d bytes\r\n", pTablesSize[TBL_Signatures]);
            CLR_Debug::Printf("   ByteCode        = %6d bytes\r\n", pTablesSize[TBL_ByteCode]);
            CLR_Debug::Printf("\r\n\r\n");
        }
    }

#endif

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::PrepareForExecutionHelper(const char *szAssembly)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    NANOCLR_FOREACH_ASSEMBLY(*this)
    {
        if (!strcmp(szAssembly, pASSM->m_szName))
        {
            NANOCLR_CHECK_HRESULT(pASSM->PrepareForExecution());
        }
    }

    NANOCLR_FOREACH_ASSEMBLY_END();

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::PrepareForExecution()
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
    CLR_EE_DBG_SET(BreakpointsDisabled);
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)

#if !defined(NANOCLR_APPDOMAINS)
    if (g_CLR_RT_ExecutionEngine.m_outOfMemoryException == NULL)
    {
        CLR_RT_HeapBlock exception;

        NANOCLR_CHECK_HRESULT(
            g_CLR_RT_ExecutionEngine.NewObjectFromIndex(exception, g_CLR_RT_WellKnownTypes.m_OutOfMemoryException));

        g_CLR_RT_ExecutionEngine.m_outOfMemoryException = exception.Dereference();
    }
#endif

    // Load Runtime.Events to setup EventSink for other assemblies using it
    NANOCLR_CHECK_HRESULT(PrepareForExecutionHelper("nanoFramework.Runtime.Events"));

    // Load Runtime.Native for other assemblies using it
    NANOCLR_CHECK_HRESULT(PrepareForExecutionHelper("nanoFramework.Runtime.Native"));

    NANOCLR_FOREACH_ASSEMBLY(*this)
    {
        NANOCLR_CHECK_HRESULT(pASSM->PrepareForExecution());
    }
    NANOCLR_FOREACH_ASSEMBLY_END();

    NANOCLR_CLEANUP();

#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)
    CLR_EE_DBG_CLR(BreakpointsDisabled);

    g_CLR_RT_ExecutionEngine.Breakpoint_Assemblies_Loaded();
#endif //#if defined(NANOCLR_ENABLE_SOURCELEVELDEBUGGING)

    NANOCLR_CLEANUP_END();
}

//--//

bool CLR_RT_TypeSystem::MatchSignature(CLR_RT_SignatureParser &parserLeft, CLR_RT_SignatureParser &parserRight)
{
    NATIVE_PROFILE_CLR_CORE();
    if (parserLeft.Type != parserRight.Type)
        return false;
    if (parserLeft.Flags != parserRight.Flags)
        return false;

    return MatchSignatureDirect(parserLeft, parserRight, false);
}

bool CLR_RT_TypeSystem::MatchSignatureDirect(
    CLR_RT_SignatureParser &parserLeft,
    CLR_RT_SignatureParser &parserRight,
    bool fIsInstanceOfOK)
{
    NATIVE_PROFILE_CLR_CORE();
    while (true)
    {
        int iAvailLeft = parserLeft.Available();
        int iAvailRight = parserRight.Available();

        if (iAvailLeft != iAvailRight)
            return false;

        if (!iAvailLeft)
            return true;

        CLR_RT_SignatureParser::Element resLeft;
        if (FAILED(parserLeft.Advance(resLeft)))
            return false;
        CLR_RT_SignatureParser::Element resRight;
        if (FAILED(parserRight.Advance(resRight)))
            return false;

        if (!MatchSignatureElement(
            resLeft, 
            resRight, 
            parserLeft,
            parserRight,
            fIsInstanceOfOK))
            return false;
    }

    return true;
}

bool CLR_RT_TypeSystem::MatchSignatureElement(
    CLR_RT_SignatureParser::Element &resLeft,
    CLR_RT_SignatureParser::Element &resRight,
    CLR_RT_SignatureParser& parserLeft,
    CLR_RT_SignatureParser& parserRight,
    bool fIsInstanceOfOK)
{
    NATIVE_PROFILE_CLR_CORE();
    if (fIsInstanceOfOK)
    {
        CLR_RT_ReflectionDef_Index indexLeft;
        CLR_RT_TypeDescriptor descLeft;
        CLR_RT_ReflectionDef_Index indexRight;
        CLR_RT_TypeDescriptor descRight;

        indexLeft.m_kind = REFLECTION_TYPE;
        indexLeft.m_levels = resLeft.Levels;
        indexLeft.m_data.m_type = resLeft.Class;

        indexRight.m_kind = REFLECTION_TYPE;
        indexRight.m_levels = resRight.Levels;
        indexRight.m_data.m_type = resRight.Class;

        if (FAILED(descLeft.InitializeFromReflection(indexLeft)))
            return false;
        if (FAILED(descRight.InitializeFromReflection(indexRight)))
            return false;

        if (!CLR_RT_ExecutionEngine::IsInstanceOf(descRight, descLeft, false))
            return false;
    }
    else
    {
        if (resLeft.IsByRef != resRight.IsByRef)
        {
            return false;
        }
        if (resLeft.Levels != resRight.Levels)
        {
            return false;
        }
        if (resLeft.DataType != resRight.DataType)
        {
            return false;
        }
        if (resLeft.Class.m_data != resRight.Class.m_data)
        {
            return false;
        }
        if ((resLeft.DataType == DATATYPE_MVAR && resLeft.DataType == DATATYPE_MVAR) &&
            (resLeft.GenericParamPosition && resRight.GenericParamPosition))
        {
            return false;
        }
        if ((resLeft.DataType == DATATYPE_VAR && resLeft.DataType == DATATYPE_VAR) &&
            (resLeft.GenericParamPosition && resRight.GenericParamPosition))
        {
            return false;
        }
        if (parserLeft.IsGenericInst != parserRight.IsGenericInst)
        {
            return false;
        }
        if (parserLeft.IsGenericInst || parserRight.IsGenericInst)
        {
            if (resLeft.GenericParamPosition != resRight.GenericParamPosition)
            {
                return false;
            }
        }
    }

    return true;
}

//--//

HRESULT CLR_RT_TypeSystem::QueueStringToBuffer(char *&szBuffer, size_t &iBuffer, const char *szText)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if (szText)
    {
        if (CLR_SafeSprintf(szBuffer, iBuffer, "%s", szText) == false)
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_OUT_OF_MEMORY);
        }
    }
    else
    {
        szBuffer[0] = 0;
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::BuildTypeName(const CLR_RT_TypeDef_Index &cls, char *&szBuffer, size_t &iBuffer)
{
    NATIVE_PROFILE_CLR_CORE();
    return BuildTypeName(cls, szBuffer, iBuffer, CLR_RT_TypeSystem::TYPENAME_FLAGS_FULL, 0);
}

HRESULT CLR_RT_TypeSystem::BuildTypeName(
    const CLR_RT_TypeDef_Index &cls,
    char *&szBuffer,
    size_t &iBuffer,
    CLR_UINT32 flags,
    CLR_UINT32 levels)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_TypeDef_Instance inst;
    CLR_RT_Assembly *assm;
    const CLR_RECORD_TYPEDEF *td;
    bool fFullName;

    if (inst.InitializeFromIndex(cls) == false)
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);

    assm = inst.m_assm;
    td = inst.m_target;
    fFullName = flags & CLR_RT_TypeSystem::TYPENAME_FLAGS_FULL;

    if (fFullName && td->HasValidEnclosingType())
    {
        CLR_RT_TypeDef_Index clsSub;
        clsSub.Set(inst.Assembly(), td->EnclosingTypeIndex());

        NANOCLR_CHECK_HRESULT(BuildTypeName(clsSub, szBuffer, iBuffer, flags, 0));

        NANOCLR_CHECK_HRESULT(QueueStringToBuffer(
            szBuffer,
            iBuffer,
            (flags & CLR_RT_TypeSystem::TYPENAME_NESTED_SEPARATOR_DOT) ? "." : "+"));
    }

    if (fFullName && td->NameSpace != CLR_EmptyIndex)
    {
        const char *szNameSpace = assm->GetString(td->NameSpace);

        if (szNameSpace[0])
        {
            NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, szNameSpace));
            NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, "."));
        }
    }

    NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, assm->GetString(td->Name)));

    while (levels-- > 0)
    {
        NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, "[]"));
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::BuildMethodName(const CLR_RT_MethodDef_Index &md, const CLR_RT_TypeSpec_Index* genericType, char *&szBuffer, size_t &iBuffer)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_MethodDef_Instance inst;
    CLR_RT_TypeDef_Instance instOwner;

    if (inst.InitializeFromIndex(md) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    if (instOwner.InitializeFromMethod(inst) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    if (genericType == NULL)
    {
        NANOCLR_CHECK_HRESULT(BuildTypeName(instOwner, szBuffer, iBuffer));

        CLR_SafeSprintf(szBuffer, iBuffer, "::%s", inst.m_assm->GetString(inst.m_target->Name));
    }
    else
    {
        CLR_RT_SignatureParser parser;
        parser.Initialize_TypeSpec(inst.m_assm, inst.m_assm->GetTypeSpec(genericType->TypeSpec()));

        CLR_RT_SignatureParser::Element element;

        // get type
        parser.Advance(element);

        CLR_RT_TypeDef_Index typeDef;
        typeDef.m_data = element.Class.m_data;

        BuildTypeName(typeDef, szBuffer, iBuffer);

        NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, "<"));

        for (int i = 0; i < parser.GenParamCount; i++)
        {
            parser.Advance(element);

            NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, c_CLR_RT_DataTypeLookup[element.DataType].m_name));

            if (i + 1 < parser.GenParamCount)
            {
                NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, ","));
            }
        }

        CLR_SafeSprintf(szBuffer, iBuffer, ">::%s", inst.m_assm->GetString(inst.m_target->Name));
    }

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::BuildFieldName(const CLR_RT_FieldDef_Index &fd, char *&szBuffer, size_t &iBuffer)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_FieldDef_Instance inst;
    CLR_RT_TypeDef_Instance instOwner;

    if (inst.InitializeFromIndex(fd) == false)
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    if (instOwner.InitializeFromField(inst) == false)
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);

    NANOCLR_CHECK_HRESULT(BuildTypeName(instOwner, szBuffer, iBuffer));

    CLR_SafeSprintf(szBuffer, iBuffer, "::%s", inst.m_assm->GetString(inst.m_target->Name));

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::BuildMethodRefName(const CLR_RT_MethodRef_Index &method, char*& szBuffer, size_t& iBuffer)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_Assembly* assembly = g_CLR_RT_TypeSystem.m_assemblies[method.Assembly() - 1];

    const CLR_RT_MethodRef_CrossReference memberCrossRef = assembly->m_pCrossReference_MethodRef[method.Method()];
    const CLR_RECORD_METHODREF* methodRef = assembly->GetMethodRef(method.Method());

    if (memberCrossRef.GenericType.m_data == CLR_EmptyToken)
    {
        // this is a MethodRef belonging to another assembly

        CLR_RT_MethodDef_Instance mdInstance;
        mdInstance.m_data = memberCrossRef.Target.m_data;
        mdInstance.m_assm = g_CLR_RT_TypeSystem.m_assemblies[mdInstance.Assembly() - 1];
        mdInstance.m_target = mdInstance.m_assm->GetMethodDef(mdInstance.Method());

        CLR_RT_TypeDef_Index typeOwner;
        typeOwner.Set(mdInstance.Assembly(), mdInstance.m_assm->m_pCrossReference_MethodDef[mdInstance.Method()].GetOwner());

        NANOCLR_CHECK_HRESULT(BuildTypeName(typeOwner, szBuffer, iBuffer));

        CLR_SafeSprintf(szBuffer, iBuffer, "::%s", mdInstance.m_assm->GetString(mdInstance.m_target->Name));
    }
    else
    {
        // this is a MethodRef for a generic type

        CLR_RT_SignatureParser parser;
        parser.Initialize_TypeSpec(assembly, assembly->GetTypeSpec(memberCrossRef.GenericType.TypeSpec()));

        CLR_RT_SignatureParser::Element element;

        // get type
        parser.Advance(element);

        CLR_RT_TypeDef_Index typeDef;
        typeDef.m_data = element.Class.m_data;

        BuildTypeName(typeDef, szBuffer, iBuffer);

        CLR_SafeSprintf(szBuffer, iBuffer, "<");

        for (int i = 0; i < parser.GenParamCount; i++)
        {
            parser.Advance(element);

            CLR_SafeSprintf(szBuffer, iBuffer, c_CLR_RT_DataTypeLookup[element.DataType].m_name);

            if (i + 1 < parser.GenParamCount)
            {
                CLR_SafeSprintf(szBuffer, iBuffer, ",");
            }
        }

        CLR_SafeSprintf(szBuffer, iBuffer, ">::%s", assembly->GetString(methodRef->Name));
    }


    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_TypeSystem::BuildMethodName(const CLR_RT_MethodSpec_Index &ms, const CLR_RT_TypeSpec_Index* genericType, char *&szBuffer, size_t &iBuffer)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    CLR_RT_MethodSpec_Instance inst;

    if (inst.InitializeFromIndex(ms) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    CLR_RT_Assembly* assembly = g_CLR_RT_TypeSystem.m_assemblies[ms.Assembly() - 1];

    const CLR_RECORD_METHODSPEC* msRecord = inst.m_assm->GetMethodSpec(ms.Method());

    CLR_RT_SignatureParser parser;
    parser.Initialize_TypeSpec(inst.m_assm, inst.m_assm->GetTypeSpec(msRecord->Container));

    CLR_RT_SignatureParser::Element element;

    // get type
    parser.Advance(element);

    CLR_RT_TypeDef_Index typeDef;
    typeDef.m_data = element.Class.m_data;

    CLR_RT_TypeDef_Instance instOwner;

    if (instOwner.InitializeFromIndex(typeDef) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    BuildTypeName(typeDef, szBuffer, iBuffer);

    CLR_SafeSprintf(szBuffer, iBuffer, "<");

    parser.Initialize_MethodSignature(&inst);

    for (int i = 0; i < parser.GenParamCount; i++)
    {
        parser.Advance(element);

        NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, c_CLR_RT_DataTypeLookup[element.DataType].m_name));

        if (i + 1 < parser.GenParamCount)
        {
            NANOCLR_CHECK_HRESULT(QueueStringToBuffer(szBuffer, iBuffer, ","));
        }
    }

    CLR_SafeSprintf(szBuffer, iBuffer, ">::");

    switch (msRecord->MethodKind())
    {
        case TBL_MethodDef:
        {
            const CLR_RECORD_METHODDEF* md = assembly->GetMethodDef(msRecord->MethodIndex());
            CLR_SafeSprintf(szBuffer, iBuffer, "%s", assembly->GetString(md->Name));
            break;
        }
        case TBL_MethodRef:
        {
            const CLR_RECORD_METHODREF* methodRef = assembly->GetMethodRef(msRecord->MethodIndex());
            const CLR_RT_MethodRef_CrossReference memberCrossRef = assembly->m_pCrossReference_MethodRef[msRecord->MethodIndex()];

            CLR_RT_MethodDef_Instance mdInstance;
            mdInstance.m_data = memberCrossRef.Target.m_data;
            mdInstance.m_assm = g_CLR_RT_TypeSystem.m_assemblies[mdInstance.Assembly() - 1];
            mdInstance.m_target = mdInstance.m_assm->GetMethodDef(mdInstance.Method());

            CLR_SafeSprintf(szBuffer, iBuffer, "%s", mdInstance.m_assm->GetString(mdInstance.CrossReference().GetOwner()));
            break;
        }
        default:
            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    NANOCLR_NOCLEANUP();
}

//--//

bool CLR_RT_TypeSystem::FindVirtualMethodDef(
    const CLR_RT_TypeDef_Index &cls,
    const CLR_RT_MethodDef_Index &calleeMD,
    CLR_RT_MethodDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_MethodDef_Instance calleeInst;

    if (calleeInst.InitializeFromIndex(calleeMD))
    {
        const char *calleeName = calleeInst.m_assm->GetString(calleeInst.m_target->Name);

        CLR_RT_TypeDef_Instance inst;
        inst.InitializeFromMethod(calleeInst);

        if ((inst.m_target->Flags & CLR_RECORD_TYPEDEF::TD_Semantics_Mask) ==
            CLR_RECORD_TYPEDEF::TD_Semantics_Interface)
        {
            //
            // It's an interface method, it could be that the class is implementing explicitly the method.
            // Prepend the Interface name to the method name and try again.
            //
            char rgBuffer[512];
            char *szBuffer = rgBuffer;
            size_t iBuffer = MAXSTRLEN(rgBuffer);

            BuildTypeName(
                inst,
                szBuffer,
                iBuffer,
                CLR_RT_TypeSystem::TYPENAME_FLAGS_FULL | CLR_RT_TypeSystem::TYPENAME_NESTED_SEPARATOR_DOT,
                0);
            QueueStringToBuffer(szBuffer, iBuffer, ".");
            QueueStringToBuffer(szBuffer, iBuffer, calleeName);

            if (FindVirtualMethodDef(cls, calleeMD, rgBuffer, index))
                return true;
        }

        if (FindVirtualMethodDef(cls, calleeMD, calleeName, index))
            return true;
    }

    index.Clear();

    return false;
}

bool CLR_RT_TypeSystem::FindVirtualMethodDef(
    const CLR_RT_TypeDef_Index &cls,
    const CLR_RT_MethodDef_Index &calleeMD,
    const char *calleeName,
    CLR_RT_MethodDef_Index &index)
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_RT_TypeDef_Instance clsInst;
    clsInst.InitializeFromIndex(cls);
    CLR_RT_MethodDef_Instance calleeInst;
    calleeInst.InitializeFromIndex(calleeMD);

    const CLR_RECORD_METHODDEF *calleeMDR = calleeInst.m_target;
    CLR_UINT8 calleeArgumentsCount = calleeMDR->ArgumentsCount;

    while (NANOCLR_INDEX_IS_VALID(clsInst))
    {
        CLR_RT_Assembly *targetAssm = clsInst.m_assm;
        const CLR_RECORD_TYPEDEF *targetTDR = clsInst.m_target;
        const CLR_RECORD_METHODDEF *targetMDR = targetAssm->GetMethodDef(targetTDR->FirstMethod);
        int num = targetTDR->VirtualMethodCount + targetTDR->InstanceMethodCount;

        for (int i = 0; i < num; i++, targetMDR++)
        {
            if (targetMDR == calleeMDR)
            {
                // Shortcut for identity.
                index = calleeInst;
                return true;
            }

            if (targetMDR->ArgumentsCount == calleeArgumentsCount &&
                (targetMDR->Flags & CLR_RECORD_METHODDEF::MD_Abstract) == 0)
            {
                const char *targetName = targetAssm->GetString(targetMDR->Name);

                if (!strcmp(targetName, calleeName))
                {
                    CLR_RT_SignatureParser parserLeft;
                    parserLeft.Initialize_MethodSignature(&calleeInst);
                    CLR_RT_SignatureParser parserRight;
                    parserRight.Initialize_MethodSignature(targetAssm, targetMDR);

                    if (CLR_RT_TypeSystem::MatchSignature(parserLeft, parserRight))
                    {
                        index.Set(targetAssm->m_index, i + targetTDR->FirstMethod);

                        return true;
                    }
                }
            }
        }

        clsInst.SwitchToParent();
    }

    index.Clear();

    return false;
}

nanoClrDataType CLR_RT_TypeSystem::MapElementTypeToDataType(CLR_UINT32 et)
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RT_DataTypeLookup *ptr = c_CLR_RT_DataTypeLookup;

    for (CLR_UINT32 num = 0; num < DATATYPE_FIRST_INVALID; num++, ptr++)
    {
        if (ptr->m_convertToElementType == et)
            return (nanoClrDataType)num;
    }

    if (et == ELEMENT_TYPE_I)
        return DATATYPE_I4;
    if (et == ELEMENT_TYPE_U)
        return DATATYPE_U4;

    return DATATYPE_FIRST_INVALID;
}

CLR_UINT32 CLR_RT_TypeSystem::MapDataTypeToElementType(nanoClrDataType dt)
{
    NATIVE_PROFILE_CLR_CORE();
    return c_CLR_RT_DataTypeLookup[dt].m_convertToElementType;
}

//--//

void CLR_RT_AttributeEnumerator::Initialize(CLR_RT_Assembly *assm)
{
    NATIVE_PROFILE_CLR_CORE();
    m_assm = assm;   // CLR_RT_Assembly*            m_assm;
    m_ptr = NULL;    // const CLR_RECORD_ATTRIBUTE* m_ptr;
    m_num = 0;       // int                         m_num;
                     // CLR_RECORD_ATTRIBUTE        m_data;
    m_match.Clear(); // CLR_RT_MethodDef_Index      m_match;
}

void CLR_RT_AttributeEnumerator::Initialize(const CLR_RT_TypeDef_Instance &inst)
{
    NATIVE_PROFILE_CLR_CORE();
    m_data.OwnerType = TBL_TypeDef;
    m_data.ownerIndex = inst.Type();

    Initialize(inst.m_assm);
}

void CLR_RT_AttributeEnumerator::Initialize(const CLR_RT_FieldDef_Instance &inst)
{
    NATIVE_PROFILE_CLR_CORE();
    m_data.OwnerType = TBL_FieldDef;
    m_data.ownerIndex = inst.Field();

    Initialize(inst.m_assm);
}

void CLR_RT_AttributeEnumerator::Initialize(const CLR_RT_MethodDef_Instance &inst)
{
    NATIVE_PROFILE_CLR_CORE();
    m_data.OwnerType = TBL_MethodDef;
    m_data.ownerIndex = inst.Method();

    Initialize(inst.m_assm);
}

bool CLR_RT_AttributeEnumerator::Advance()
{
    NATIVE_PROFILE_CLR_CORE();
    const CLR_RECORD_ATTRIBUTE *ptr = m_ptr;
    int num = m_num;
    CLR_UINT32 key = m_data.Key();
    bool fRes = false;

    if (ptr == NULL)
    {
        ptr = m_assm->GetAttribute(0) - 1;
        num = m_assm->m_pTablesSize[TBL_Attributes];
    }

    while (num)
    {
        ptr++;
        num--;

        if (ptr->Key() == key)
        {
            CLR_INDEX tk = ptr->constructor;
            // check TYPEDEF
            if (tk & 0x8000)
            {
                m_match = m_assm->m_pCrossReference_MethodRef[tk & 0x7FFF].Target;
            }
            else
            {
                m_match.Set(m_assm->m_index, tk);
            }

            m_blob = m_assm->GetSignature(ptr->data);

            fRes = true;
            break;
        }
    }

    m_ptr = ptr;
    m_num = num;

    return fRes;
}

void CLR_RT_AttributeEnumerator::GetCurrent(CLR_RT_TypeDef_Instance *instTD)
{
    CLR_RT_MethodDef_Instance md;

    md.InitializeFromIndex(m_match);
    instTD->InitializeFromMethod(md);
}

bool CLR_RT_AttributeEnumerator::MatchNext(
    const CLR_RT_TypeDef_Instance *instTD,
    const CLR_RT_MethodDef_Instance *instMD)
{
    NATIVE_PROFILE_CLR_CORE();
    while (Advance())
    {
        if (instMD)
        {
            if (m_match.m_data != instMD->m_data)
                continue;
        }

        if (instTD)
        {
            CLR_RT_MethodDef_Instance md;
            CLR_RT_TypeDef_Instance td;

            md.InitializeFromIndex(m_match);
            td.InitializeFromMethod(md);

            if (td.m_data != instTD->m_data)
                continue;
        }

        return true;
    }

    return false;
}

//--//

HRESULT CLR_RT_AttributeParser::Initialize(const CLR_RT_AttributeEnumerator &en)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if (m_md.InitializeFromIndex(en.m_match) == false || m_td.InitializeFromMethod(m_md) == false)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
    }

    m_parser.Initialize_MethodSignature(&m_md);
    m_parser.Advance(m_res); // Skip return value.

    m_assm = en.m_assm;
    m_blob = en.m_blob;

    m_currentPos = 0;
    m_fixed_Count = m_md.m_target->ArgumentsCount - 1;
    m_named_Count = -1;
    m_constructorParsed = false;
    m_mdIndex = en.m_match;

    NANOCLR_NOCLEANUP();
}

HRESULT CLR_RT_AttributeParser::Next(Value *&res)
{
    NATIVE_PROFILE_CLR_CORE();
    NANOCLR_HEADER();

    if (m_currentPos == m_fixed_Count)
    {
        NANOCLR_READ_UNALIGNED_UINT16(m_named_Count, m_blob);
    }

    if (m_fixed_Count == 0 && m_named_Count == 0 && !m_constructorParsed)
    {
        // Attribute class has no fields, no properties and only default constructor

        m_lastValue.m_mode = Value::c_DefaultConstructor;
        m_lastValue.m_name = NULL;

        NANOCLR_CHECK_HRESULT(g_CLR_RT_ExecutionEngine.NewObject(m_lastValue.m_value, m_td));

        res = &m_lastValue;

        m_constructorParsed = true;

        NANOCLR_SET_AND_LEAVE(S_OK);
    }
    else if ((m_currentPos < m_fixed_Count) && !m_constructorParsed)
    {
        // Attribute class has a constructor

        m_lastValue.m_mode = Value::c_ConstructorArgument;
        m_lastValue.m_name = NULL;

        ////////////////////////////////////////////////
        // need to read the arguments from the blob

        NANOCLR_CHECK_HRESULT(m_parser.Advance(m_res));
        //
        // Skip value info.
        //
        m_blob += sizeof(CLR_UINT8);

        const CLR_RT_DataTypeLookup &dtl = c_CLR_RT_DataTypeLookup[m_res.DataType];

        if (dtl.m_flags & CLR_RT_DataTypeLookup::c_Numeric)
        {
            // size of value
            CLR_UINT32 size = dtl.m_sizeInBytes;

            NANOCLR_CHECK_HRESULT(
                g_CLR_RT_ExecutionEngine.NewObjectFromIndex(m_lastValue.m_value, g_CLR_RT_WellKnownTypes.m_TypeStatic));

            // need to setup reflection and data type Id to properly setup the object
            m_lastValue.m_value.SetReflection(*dtl.m_cls);

            m_lastValue.m_value.SetDataId(CLR_RT_HEAPBLOCK_RAW_ID(m_res.DataType, 0, 1));

            // because this is a numeric object, performa a raw copy of the numeric value data from the blob to the
            // return value
            memcpy((CLR_UINT8 *)&m_lastValue.m_value.NumericByRef(), m_blob, size);
            m_blob += size;
        }
        else if (m_res.DataType == DATATYPE_STRING)
        {
            CLR_UINT32 tk;
            NANOCLR_READ_UNALIGNED_UINT16(tk, m_blob);

            CLR_RT_HeapBlock_String::CreateInstance(m_lastValue.m_value, CLR_TkFromType(TBL_Strings, tk), m_assm);
        }
        else
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
        }

        res = &m_lastValue;

        m_constructorParsed = true;

        NANOCLR_SET_AND_LEAVE(S_OK);
    }
    else if (m_currentPos < m_fixed_Count + m_named_Count && !m_constructorParsed)
    {
        // Attribute class has named fields

        CLR_UINT32 kind;
        NANOCLR_READ_UNALIGNED_UINT8(kind, m_blob);

        m_lastValue.m_name = GetString();

        if (kind == SERIALIZATION_TYPE_FIELD)
        {
            CLR_RT_FieldDef_Index fd;
            CLR_RT_FieldDef_Instance inst;

            m_lastValue.m_mode = Value::c_NamedField;

            NANOCLR_CHECK_HRESULT(g_CLR_RT_ExecutionEngine.FindFieldDef(m_td, m_lastValue.m_name, fd));

            inst.InitializeFromIndex(fd);

            m_parser.Initialize_FieldDef(inst.m_assm, inst.m_target);
        }
        else
        {
            m_lastValue.m_mode = Value::c_NamedProperty;

            //
            // it's supposed to reach here when there is an attribute contructor
            // but that is already handled upwards
            // leaving this here waiting for a special case that hits here (if there is one...)
            //
            NANOCLR_SET_AND_LEAVE(CLR_E_NOT_SUPPORTED);
        }
    }
    else
    {
        res = NULL;
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    NANOCLR_CHECK_HRESULT(m_parser.Advance(m_res));

    res = &m_lastValue;

    //
    // Check for Enums.
    //
    if (m_res.DataType == DATATYPE_VALUETYPE)
    {
        CLR_RT_TypeDef_Instance td;
        td.InitializeFromIndex(m_res.Class);

        if ((td.m_target->Flags & CLR_RECORD_TYPEDEF::TD_Semantics_Mask) == CLR_RECORD_TYPEDEF::TD_Semantics_Enum)
        {
            m_res.DataType = (nanoClrDataType)td.m_target->DataType;
        }
    }

    //
    // Skip value info.
    //
    m_blob += sizeof(CLR_UINT8);

    {
        const CLR_RT_DataTypeLookup &dtl = c_CLR_RT_DataTypeLookup[m_res.DataType];

        if (dtl.m_flags & CLR_RT_DataTypeLookup::c_Numeric)
        {
            // need to setup reflection and data type Id to properly setup the object
            m_lastValue.m_value.SetReflection(m_res.Class);

            m_lastValue.m_value.SetDataId(CLR_RT_HEAPBLOCK_RAW_ID(m_res.DataType, 0, 1));

            CLR_UINT32 size = dtl.m_sizeInBytes;

            memcpy(&m_lastValue.m_value.NumericByRef(), m_blob, size);
            m_blob += size;
        }
        else if (m_res.DataType == DATATYPE_STRING)
        {
            CLR_UINT32 tk;
            NANOCLR_READ_UNALIGNED_UINT16(tk, m_blob);

            CLR_RT_HeapBlock_String::CreateInstance(m_lastValue.m_value, CLR_TkFromType(TBL_Strings, tk), m_assm);
        }
        else
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_WRONG_TYPE);
        }
    }

    m_lastValue.m_pos = m_currentPos++;

    NANOCLR_NOCLEANUP();
}

const char *CLR_RT_AttributeParser::GetString()
{
    NATIVE_PROFILE_CLR_CORE();
    CLR_UINT32 tk;
    NANOCLR_READ_UNALIGNED_UINT16(tk, m_blob);

    return m_assm->GetString(tk);
}
