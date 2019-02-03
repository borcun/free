## Future Airborne Capability Environment

 - FACE was formed in 2010 to define an open avionics environment for all military
   airborne platform types.
 
 - Government-industry software standard and business strategy for acquisition
   of affordable software systems that promote innovation and rapid integration of
   portable capabilities across programs.
 
 - The use of standard interfaces that will lead to reuse of capabilities.

 - Portability of applications across multiple FACE systems and vendors.

 - Making safety-critical computing operations more robust, interoperable, portable,
   and secure.
   
 - The latest edition promotes application interoperability and portability with
   enhanced requirements for exchanging data among FACE compontents, including a 
   formally specified data model and emphasis on defining common language requirements
   for the standard. This architecture defines standardized interfaces to allow
   software compontents to be moved between systems, including those developed by
   different vendors. The standardized interfaces follow a data architecture to ensure
   the data communicated between the software components is fully described to facilitate
   their integration on new systems.
   
 - The FACE Reference Architecture is composed of logical segments where variance occurs.
   The structure created by connecting these segments together is the foundation of FACE
   Reference Architecture. The five segment of the FACE Reference Architecture are;

    1. Operating System Segment (OSS)
	2. I/O Services Segment (IOSS)
	3. Platform-Specific Services Segment (PSSS)
	4. Transport Service Segment (TSS)
	5. Portable Components Segment (PCS)
  
   FACE Reference Architecture defines a set of standardized interfaces providing connections 
   between the FACE architectural segments.
   
 - The FACE Reference Architecture defines three FACE OSS Profiles tailoring the Operating System (OS) 
   Application Programming Interfaces (APIs), programming languages, programming language features, 
   run-times, frameworks, and graphics capabilities to meet the requirements of software components 
   for differing levels of criticality. The three Profiles are Security, Safety, and General Purpose. 
   The Security Profile constrains the OS APIs to a minimal useful set allowing assessment for 
   high-assurance security functions executing as a single process. The Safety Profile is less 
   restrictive than the Security Profile and constrains the OS APIs to those that have a safety 
   certification pedigree. The General Purpose Profile is the least constrained profile and supports 
   OS APIs meeting real-time deterministic or non-real-time, non-deterministic requirements depending 
   on the system or subsystem implementation. 

 - The FACE Data Architecture defines the FACE Data Model Language (including the language binding 
   specification), Query and Template language, FACE Shared Data Model (SDM) and the rules of 
   construction of the Unit of Portability (UoP) Supplied Model (USM). Each PCS Unit of Conformance 
   (UoC), PSSS UoC, or TSS UoC providing using TS Interfaces is accompanied by a USM consistent with 
   the FACE SDM and defines its interfaces in terms of the FACE Data Model Language. A Domain-Specific 
   Data Model (DSDM) captures content relevant to a domain of interest and can be used as a basis for 
   USMs.

### Operating System Segment
 The OSS is where foundational system services and vendor-supplied software reside. An OSS
 UoC provides and controls access to the computing platform. An OSS UoC supports the
 execution of all FACE UoCs and hosts various operating system, integration, and low-level
 health monitoring interfaces. An OSS UoC can also optionally provide external networking
 capabilities, Programming Language Run-Times, Component Framework, Life Cycle Management, 
 and Configuration Services capabilities.

### I/O Service Segment
 The IOSS is where normalization of vendor-supplied interface hardware device drivers occurs.
 IOSS UoCs provide the abstraction of the interface hardware and drivers from the PSSS UoCs.
 This allows the PSSS UoCs to focus on the interface data and not the hardware and driver
 specifics.

### Platform-Specific Service Segment
 The PSSS is comprised of sub-segments including Platform-Specific Device Services, Platform-
 Specific Common Services, and Platform-Specific Graphics Services.

#### Platform-Specific Device Services
 Platform-Specific Device Services (PSDS) are where management of data and translation
 between platform-unique Interface Control Documents (ICDs) and the FACE Data Model
 occurs.

#### Platform-Specific Common Services
 Platform-Specific Common Services (PSCS) are comprised of higher-level services including
 Logging Services, Device Protocol Mediation (DPM) Services, Streaming Media, Health
 Monitoring and Fault Management (HMFM), and Configuration Services.

#### Platform-Specific Graphics Services
 Platform-Specific Graphics Services (PSGS) is where presentation management occurs. PSGS
 abstracts the interface specifics of Graphics Processing Units (GPU) another graphics devices
 from software components within the FACE Reference Architecture.

### Transport Services Segment
 The TSS is comprised of communication services. The TSS abstracts transport mechanisms and
 data access from software components facilitating integration into disparate architectures and
 platforms using different transports. TSS UoCs are responsible for data distribution between
 PCS and/or PSSS UoCs. TSS capabilities include, but are not limited to, distribution and routing,
 prioritization, addressability, association, abstraction, transformation, and component state
 persistence of software component interface information.

### Portable Components Segment
 The PCS is comprised of software components providing capabilities and/or business logic. PCS
 components are intended to remain agnostic from hardware and sensors. Additionally, these
 components are not tied to any data transport or operating system implementations to meet
 objectives of portability and interoperability.
